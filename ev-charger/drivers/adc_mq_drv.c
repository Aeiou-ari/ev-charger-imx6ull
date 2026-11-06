#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <asm/uaccess.h>

/*1. i.MX6ULL 片内 ADC 为 12bit, 参考电压 3.3V(由 reg_vref_adc 提供) */
#define MQ_VREF_MV			3300
#define MQ_ADC_MAX			4095

/*2. MQ 模块板载负载电阻 RL(单位:K欧), 常见模块为 5K 或 10K, 按实际模块修改 */
#define MQ_RL_KOHM			10

/*2.1 MQ 模块的供电电压(单位:mV), 必须与实际接线一致: 接 3.3V 写 3300, 接 5V 写 5000
 *    它不是 ADC 参考电压, 两者混用会把 Rs 算小
 */
#define MQ_VCC_MV			3300

/*3. 洁净空气下标定出来的 R0(单位:K欧), 上电预热 24h 后重新标定最准 */
#define MQ_R0_KOHM			10

/*3.1 MQ-2 洁净空气系数: R0 = Rs(洁净空气) / 9.8, 放大 10 倍存成整数 */
#define MQ_CLEAN_AIR_FACTOR	98

/*3.2 基线缓慢回落速率: 每次采样回落 基线/1000000
 *    约合每小时回落基线的 0.7%, 既能跟上长期漂移, 又不会被短期波动带跑
 *    注意: 直接写 "基线 -= 基线/10000" 在基线较小时会退化成每次减 1, 掉得飞快
 */
#define MQ_BASE_DECAY		1000000

/*3.3 电阻内部统一放大 100 倍(单位 0.01K欧)再做整数运算
 *    Rs 只有几 K 时, 直接取整会把精度丢光(R0 = Rs/9.8 更会被截成 0), 必须放大
 */
#define MQ_RS_SCALE			100

/*3.4 取值上限, 保证 32 位整数运算不溢出
 *    ARM32 内核不导出 64 位除法 __aeabi_ldivmod, 模块里绝不能用 long long 除法,
 *    否则 insmod 直接报 Unknown symbol, 只能靠限制取值范围把结果压在 int 以内
 */
#define MQ_RL_MAX_KOHM		1000
#define MQ_VCC_MAX_MV		5000
#define MQ_RS_MAX_X100		1000000

/*4. MQ 输出抖动较大, 多次采样求平均 */
#define MQ_SAMPLE_CNT		8
#define MQ_SAMPLE_DELAY		5

/*5. 返回给应用层的数据包 */
struct mq_data {
	int raw;		/* ADC 原始值 		0 ~ 4095 	*/
	int mv;			/* 传感器输出电压 	mV 			*/
	int rs_x100;	/* 传感器体电阻 Rs, 放大 100 倍, 单位 0.01K欧 */
	int ppm;		/* 估算气体浓度 	ppm 		*/
	int percent;	/* ADC 满量程占比 	0 ~ 100 	*/
	int alarm;		/* DO 比较器输出, 未接 DO 时为 -1 */
	int ratio;		/* Rs/R0 放大 100 倍, 洁净空气约 980 */
	int base_x100;	/* 自动基线 Rs, 同样放大 100 倍 	*/
};

/*6. MQ-2 灵敏度曲线: Rs/R0(放大 100 倍) -> ppm, 按降序排列
 *   该表来自数据手册曲线的近似取值, 只作参考, 精准测量需用标准气体重新标定
 */
struct mq_curve_point {
	int ratio_x100;
	int ppm;
};

static const struct mq_curve_point mq2_curve[] = {
	{1000,    20},
	{ 500,   100},
	{ 260,   200},
	{ 210,   500},
	{ 170,  1000},
	{ 135,  2000},
	{ 115,  3000},
	{  95,  5000},
	{  75, 10000},
};

static struct iio_channel *pmq_chan = NULL;
static int mq_rl_kohm = MQ_RL_KOHM;
static int mq_vcc_mv = MQ_VCC_MV;
static int mq_r0_kohm = MQ_R0_KOHM;
static int mq_do_gpio = -1;

/*8.1 运行期可调参数: 不想改设备树时, 用这些内核模块参数调
 *    insmod adc_mq_drv.ko mq_vcc_mv=5000
 *    或加载后 echo 5000 > /sys/module/adc_mq_drv/parameters/mq_vcc_mv
 *    可调: mq_rl_kohm / mq_vcc_mv / mq_r0_kohm / mq_auto_cal / mq_rs_base_x100
 */
module_param(mq_rl_kohm, int, 0644);
module_param(mq_vcc_mv, int, 0644);
module_param(mq_r0_kohm, int, 0644);

/*8.2 自动标定开关: 1=驱动自己把"最洁净"的一次读数当基线(默认开)
 *    关掉后才使用 mq,r0-kohm / r0_kohm
 */
static int mq_auto_cal = 1;
module_param(mq_auto_cal, int, 0644);

/*8.3 自动基线: 记录到的最大 Rs(越干净 Rs 越大), 0 表示还没建立
 *    写入 0 可以强制重新标定:
 *    echo 0 > /sys/module/adc_mq_drv/parameters/mq_rs_base_x100
 */
static int mq_rs_base_x100 = 0;
module_param(mq_rs_base_x100, int, 0644);

/*8.4 基线回落用的小累加器(不够一个单位时先攒着) */
static int mq_base_decay_acc = 0;

/*7. 根据 Rs/R0 查表 + 线性插值求出 ppm */
static int mq_ratio_to_ppm(int ratio_x100)
{
	int i = 0;
	int cnt = 0;
	int dr = 0;
	int dp = 0;

	cnt = ARRAY_SIZE(mq2_curve);

	/*高于曲线最大值, 浓度低于检测下限 */
	if (ratio_x100 >= mq2_curve[0].ratio_x100) {
		return mq2_curve[0].ppm;
	}

	/*低于曲线最小值, 浓度超出量程 */
	if (ratio_x100 <= mq2_curve[cnt - 1].ratio_x100) {
		return mq2_curve[cnt - 1].ppm;
	}

	for (i = 0; i < cnt - 1; i++) {
		if (ratio_x100 <= mq2_curve[i].ratio_x100 &&
			ratio_x100 >= mq2_curve[i + 1].ratio_x100) {
			dr = mq2_curve[i].ratio_x100 - mq2_curve[i + 1].ratio_x100;
			dp = mq2_curve[i + 1].ppm - mq2_curve[i].ppm;

			return mq2_curve[i].ppm +
				   (mq2_curve[i].ratio_x100 - ratio_x100) * dp / dr;
		}
	}

	return mq2_curve[cnt - 1].ppm;
}

/*8. 读取 ADC 原始值(多次采样求平均) */
static int mq_read_adc_raw(int *praw)
{
	int i = 0;
	int ret = 0;
	int val = 0;
	int sum = 0;

	for (i = 0; i < MQ_SAMPLE_CNT; i++) {
		ret = iio_read_channel_raw(pmq_chan, &val);
		if (ret < 0) {
			pr_info("iio_read_channel_raw failed\n");
			return -1;
		}

		sum += val;
		mdelay(MQ_SAMPLE_DELAY);
	}

	*praw = sum / MQ_SAMPLE_CNT;

	return 0;
}

/*9. 换算: raw -> mv -> Rs -> ppm */
static int mq_get_data(struct mq_data *pdata)
{
	int ret = 0;
	int raw = 0;
	int mv = 0;
	int rs_x100 = 0;
	int ratio_x100 = 0;
	int q = 0;
	int r = 0;

	ret = mq_read_adc_raw(&raw);
	if (-1 == ret) {
		pr_info("mq_read_adc_raw failed\n");
		return -1;
	}

	if (raw < 0) {
		raw = 0;
	}
	if (raw > MQ_ADC_MAX) {
		raw = MQ_ADC_MAX;
	}

	/*1. 原始值转电压(用 ADC 参考电压, 固定 3.3V) */
	mv = raw * MQ_VREF_MV / MQ_ADC_MAX;

	/*1.1 把可调参数夹到安全范围, 防止误填大数导致乘法溢出 */
	if (mq_rl_kohm < 1) {
		mq_rl_kohm = 1;
	} else if (mq_rl_kohm > MQ_RL_MAX_KOHM) {
		mq_rl_kohm = MQ_RL_MAX_KOHM;
	}

	if (mq_vcc_mv < 1) {
		mq_vcc_mv = 1;
	} else if (mq_vcc_mv > MQ_VCC_MAX_MV) {
		mq_vcc_mv = MQ_VCC_MAX_MV;
	}

	/*2. 电压转传感器体电阻 Rs = RL * (VCC - Vout) / Vout
	 *   VCC 是 MQ 模块的供电电压, 接 5V 时必须写成 5000, 否则 Rs 会被算小
	 */
	if (mv <= 0) {
		rs_x100 = mq_rl_kohm * 100 * MQ_RS_SCALE;
	} else if (mv >= mq_vcc_mv) {
		rs_x100 = 0;
	} else {
		rs_x100 = mq_rl_kohm * (mq_vcc_mv - mv) * MQ_RS_SCALE / mv;
	}

	if (rs_x100 > MQ_RS_MAX_X100) {
		rs_x100 = MQ_RS_MAX_X100;
	}

	/*3. 求 R0
	 *   自动标定: 把"最洁净"的一次读数(即 Rs 的最大值)当基线
	 *   Rs 越大代表空气越干净, 所以遇到更大的 Rs 就抬高基线;
	 *   平时只让基线极缓慢地回落, 这样长期漂移不会把基线锁死
	 */
	if (mq_auto_cal) {
		if (mq_rs_base_x100 <= 0) {
			mq_rs_base_x100 = rs_x100;
			mq_base_decay_acc = 0;
		} else if (rs_x100 > mq_rs_base_x100) {
			/*比基线还干净, 抬高基线 */
			mq_rs_base_x100 = rs_x100;
			mq_base_decay_acc = 0;
		} else {
			/*比基线脏: 攒够一个单位才回落, 速率 基线/1000000 每次 */
			mq_base_decay_acc += mq_rs_base_x100;
			if (mq_base_decay_acc >= MQ_BASE_DECAY) {
				mq_base_decay_acc -= MQ_BASE_DECAY;
				if (mq_rs_base_x100 > 1) {
					mq_rs_base_x100 -= 1;
				}
			}
		}

		if (mq_rs_base_x100 < 1) {
			mq_rs_base_x100 = 1;
		} else if (mq_rs_base_x100 > MQ_RS_MAX_X100) {
			mq_rs_base_x100 = MQ_RS_MAX_X100;
		}

		/*4. Rs/R0 放大 100 倍
		 *   R0 = 基线 / 9.8  =>  Rs/R0 = Rs * 9.8 / 基线
		 *   再放大 100 倍就是 Rs * 980 / 基线
		 *   这里拆成 商*980 + 余数*980/基线 两步: 余数必定小于基线,
		 *   所以乘法结果最大约 1e8, 不会冲出 int, 也就用不上 64 位除法
		 *   空气最干净(Rs == 基线)时结果恒等于 980
		 */
		q = rs_x100 / mq_rs_base_x100;
		r = rs_x100 % mq_rs_base_x100;
		ratio_x100 = q * (MQ_CLEAN_AIR_FACTOR * 10) +
					 r * (MQ_CLEAN_AIR_FACTOR * 10) / mq_rs_base_x100;
	} else {
		/*手动模式: R0 直接用 mq,r0-kohm(单位 K欧) */
		if (mq_r0_kohm < 1) {
			mq_r0_kohm = 1;
		}
		ratio_x100 = rs_x100 / mq_r0_kohm;
	}

	pdata->raw = raw;
	pdata->mv = mv;
	pdata->rs_x100 = rs_x100;
	pdata->ppm = mq_ratio_to_ppm(ratio_x100);
	pdata->percent = raw * 100 / MQ_ADC_MAX;
	pdata->ratio = ratio_x100;
	pdata->base_x100 = mq_rs_base_x100;

	if (mq_do_gpio >= 0) {
		pdata->alarm = gpio_get_value(mq_do_gpio) ? 1 : 0;
	} else {
		pdata->alarm = -1;
	}

	return 0;
}

static ssize_t mq_read(struct file *fp, char __user *puser, size_t n, loff_t *off)
{
	unsigned long nret = 0;
	int ret = 0;
	struct mq_data data = {0};

	//1.读取 MQ 传感器(1.ADC采样 2.换算电压与电阻 3.查表求浓度)
	ret = mq_get_data(&data);
	if (-1 == ret) {
		pr_info("mq_get_data failed\n");
		return -1;
	}

	//2.copy_to_user
	nret = copy_to_user(puser, &data, sizeof(data));
	if (nret) {
		pr_info("copy_to_user failed\n");
		return -1;
	}

	return sizeof(data);
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = mq_read,
};

static struct miscdevice mq_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mq_misc",
	.fops = &fops,
};

static int mq_probe(struct platform_device *pdev)
{
	int ret = 0;
	u32 val = 0;
	struct device_node *pnode = NULL;

	/*1. 获取设备树里绑定的 ADC 通道 */
	pmq_chan = iio_channel_get(&pdev->dev, "mq-adc");
	if (IS_ERR(pmq_chan)) {
		pr_info("iio_channel_get(mq-adc) err = %ld\n", PTR_ERR(pmq_chan));

		/*设备树没有写 io-channel-names 时, 退化为按序号 0 取通道 */
		pmq_chan = iio_channel_get(&pdev->dev, NULL);
		if (IS_ERR(pmq_chan)) {
			ret = PTR_ERR(pmq_chan);
			pmq_chan = NULL;
			pr_info("iio_channel_get failed, err = %d\n", ret);
			pr_info("check: adc node needs #io-channel-cells = <1>\n");
			return ret;
		}
	}

	/*2. 可选参数: 负载电阻 RL / 模块供电电压 / 标定电阻 R0 */
	pnode = pdev->dev.of_node;
	if (pnode) {
		if (!of_property_read_u32(pnode, "mq,rl-kohm", &val)) {
			mq_rl_kohm = (int)val;
		}
		if (!of_property_read_u32(pnode, "mq,vcc-mv", &val)) {
			mq_vcc_mv = (int)val;
		}

		/*洁净空气里读到的 Rs, 除以洁净空气系数就是 R0 */
		if (!of_property_read_u32(pnode, "mq,rs-clean-kohm", &val)) {
			mq_r0_kohm = ((int)val) * 10 / MQ_CLEAN_AIR_FACTOR;
		}
		if (!of_property_read_u32(pnode, "mq,r0-kohm", &val)) {
			mq_r0_kohm = (int)val;
		}

		if (mq_rl_kohm <= 0) {
			mq_rl_kohm = MQ_RL_KOHM;
		}
		if (mq_vcc_mv <= 0) {
			mq_vcc_mv = MQ_VCC_MV;
		}
		if (mq_r0_kohm <= 0) {
			mq_r0_kohm = MQ_R0_KOHM;
		}

		/*3. 可选参数: DO 比较器输出引脚, 不接就不申请 */
		mq_do_gpio = of_get_named_gpio(pnode, "gpio-do", 0);
		if (gpio_is_valid(mq_do_gpio)) {
			ret = gpio_request(mq_do_gpio, "mq-do");
			if (ret) {
				pr_info("gpio_request do failed\n");
				mq_do_gpio = -1;
			} else {
				gpio_direction_input(mq_do_gpio);
			}
		} else {
			mq_do_gpio = -1;
		}
	}

	pr_info("rl = %dK, vcc = %dmV, r0 = %dK, auto_cal = %d, do_gpio = %d\n",
			mq_rl_kohm, mq_vcc_mv, mq_r0_kohm, mq_auto_cal, mq_do_gpio);
	pr_info("提示: 洁净空气预热完成后, Rs/R0 应稳定在 980 左右\n");

	/*4. 注册杂项设备 */
	ret = misc_register(&mq_misc);
	if (ret) {
		pr_info("misc_register failed\n");
		return -1;
	}

	pr_info("probe ok!\n");

	return 0;
}

static int mq_remove(struct platform_device *pdev)
{
	int ret = 0;

	if (mq_do_gpio >= 0) {
		gpio_free(mq_do_gpio);
		mq_do_gpio = -1;
	}

	if (pmq_chan) {
		iio_channel_release(pmq_chan);
		pmq_chan = NULL;
	}

	ret = misc_deregister(&mq_misc);
	if (ret) {
		pr_info("misc_deregister failed\n");
		return -1;
	}

	pr_info("remove ok!\n");

	return 0;
}

static struct of_device_id mq_of_match_table[] = {
	{.compatible = "pute,putemq"},
	{},
};

static struct platform_device_id mq_id_table[] = {
	{.name = "putemq"},
	{},
};

static struct platform_driver mq_drv = {
	.probe = mq_probe,
	.remove = mq_remove,
	.driver = {
		.name = "putemq",
		.owner = THIS_MODULE,
		.of_match_table = mq_of_match_table,
	},
	.id_table = mq_id_table,
};

static int __init mq_drv_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&mq_drv);
	if (ret) {
		pr_info("platform_driver_register failed\n");
		return -1;
	}

	return 0;
}

static void __exit mq_drv_exit(void)
{
	platform_driver_unregister(&mq_drv);

	return;
}

module_init(mq_drv_init);
module_exit(mq_drv_exit);

MODULE_LICENSE("GPL");
