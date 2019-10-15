/**

 * @defgroup SDK_TIPS  
 * 一些使用SDK的说明\n
 1.示例程序执行流程\n
   这段代码在模组主程序的main函数中调用，不能阻塞。主要是新建了一个任务，该任务是OPENCPU用户添加定制化代码的地方。
   @code
   void test_opencpu_start()
   {
	 xTaskCreate(opencpu_task_main,"opencpu",1024,NULL,TASK_PRIORITY_NORMAL,NULL);
   } 
   @endcode
   这段代码是用户任务的函数体，该任务在整个操作系统开始调度后作为一个普通任务与其他系统任务一起运行，用户的所有操作在该函数中进行，这个任务如果退出，则代表用户执行流也退出了。
   @code
   void opencpu_task_main()
   {
  		test_all_in_one();
  		vTaskDelete(NULL);
   }
   @endcode
  2.模组API函数功能测试方法
   <pre>
   在示例代码中，test_all_in_one()函数是用户执行功能测试的函数，该函数在opencpu_task_main()任务中调用，模组开机就开始执行。
   在该函数中，首先初始化了串口1，设置了串口1回调函数，然后打印了一些提示信息，然后锁定睡眠，使模组不能进入睡眠，方便测试
   以阻塞方式获取ICCID，如果没插卡，则程序不能继续往下执行
   以阻塞方式等待程序驻网(CGACT)，如果驻网不成功，则程序不能继续往下执行
   打印出驻网成功信息，循环等待用户命令。
   用户此时可在串口1输入单字符命令(A~Z,a~z,0~9)以执行相应的功能函数
  </pre>
 3.模组驻网检测方法
	<pre>
   模组驻网检测只需检测CGACT值即可，如函数opencpu_cgact()返回值为1，则说明已经能够上网：
   </pre>
 4.SDK中可使用的函数接口说明
	<pre>
		1.AT手册中已经定义的AT命令，可在ril.h中找到相同定义的，可使用。仅在ril.h中定义而未在AT手册中定义的指令不可使用。
		2.此手册中定义的函数接口或此手册说明可使用的接口
		3.C库中不涉及物理硬件的函数接口
	备注：如有其他未在以上范围，且必要的功能接口，请联系技术支持
	</pre>
 
 *@{
 */
 
 
 
/** @} */
/**

 * @defgroup FUNCTIONS  
 * opencpu SDK开放的函数接口
 *@{
 */
/**

 * @defgroup BASE_FUNCTIONS  
 * 模组的基本功能函数接口
 *@{
 */
/** @brief 使用此宏在opencpu中打印log信息. */
#define opencpu_log(fmt,arg...) printf(fmt,##arg)
/**
 *  \brief 获取sim卡EID
 *  
 *  \param [in] buf 保存EID的buffer指针
 *  \return 0:成功 -1：失败
 *  
 *  \details More details
 */
int opencpu_get_eid(unsigned char *buf);
/**
 *  \brief 读取log输出的配置
 *  
 *  \param [in] func_name "emmi":GKI log. "uls":HSL log
 *  \param [in] port 保存串口号的指针
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_read_port_config(unsigned char * func_name,serial_port_dev_t *port);
/**
 *  \brief us级延时函数
 *  
 *  \param [in] us 延时周期数
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_delay_us(int us);
/**
 *  \brief ms级延时函数
 *  
 *  \param [in] ms 延时周期数
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_delay_ms(int ms);
/**
 *  \brief 设置log的输出串口（支持四个物理串口和usb的两个虚拟串口），设置完后重启生效
 *  
 *  \param [in] func_name "emmi":GKI log. "uls":HSL log
 *  \param [in] port 要输出的串口，宏定义见serial_port.h
 *  \return Return 空
 *  
 *  \details 注意：一个串口只能输出一种log
 */
void opencpu_write_port_config(unsigned char * func_name,serial_port_dev_t port);
/**
 *  \brief 单独对模组当前IMEI和备份IMEI进行写入。写入前先执行CFUN=0
 *  
 *  \param [in] type 写入IMEI类型，0：当前IMEI 1：备份IMEI
 *  \param [in] imei_buf IMEI字符串
 *  \return Return 空
 *  
 *  \details IMEI写入有失败风险，请读出进行检验，确保写入成功
 */
void  opencpu_write_imei_custom(int type,unsigned char *imei_buf);
/**
 *  \brief cmsysctrl配置函数
 *  
 *  \param [in] op 设置引脚设置，0为STATE，1为WAKEUP-OUT
 *  \param [in] mode 设置模式值，与op共同生效；
 *  \param [in] net_light_time_ms 仅在op=0且mode=2时有意义，默认80(ms),范围40-65535(ms)，表示非注册状态下高电平持续时间；
 *  \param [in] net_light_regist_time_ms 仅在op=0且mode=2时有意义，默认80,范围40-65535(ms)，表示注册状态下高电平持续时间；
 *  \param [in] net_slake_registing_time_ms 仅在op=0且mode=2时有意义，默认800(ms)，范围40-65535(ms)，表示非注册状态下低电平持续时间；
 *  \param [in] net_slake_regist_time_ms 仅在op=0且mode=2时有意义，默认3000(ms)，范围40-65535(ms)，表示注册状态下低电平持续时间；
 *  \return 0：成功 -1：失败或者入参非法；
 *  \更多信息参阅《M5311_AT_Command_Interface_Specification.pdf》中AT+CMSYSCTRL指令说明；
 *  \
 */
int opencpu_set_cmsysctrl(int op, int mode, int net_light_time_ms, int net_light_regist_time_ms, int net_slake_registing_time_ms, int net_slake_regist_time_ms);

/**
 *  \brief cmsysctrl读函数
 *  
 *  \param [in] sys_status status状态
 *  \param [in] wakeup_out wakeup_out状态
 *  \param [in] net_light_time_ms 仅在op=0且mode=2时有意义，默认80(ms),范围40-65535(ms)，表示非注册状态下高电平持续时间；
 *  \param [in] net_light_regist_time_ms 仅在op=0且mode=2时有意义，默认80,范围40-65535(ms)，表示注册状态下高电平持续时间；
 *  \param [in] net_slake_registing_time_ms 仅在op=0且mode=2时有意义，默认800(ms)，范围40-65535(ms)，表示非注册状态下低电平持续时间；
 *  \param [in] net_slake_regist_time_ms 仅在op=0且mode=2时有意义，默认3000(ms)，范围40-65535(ms)，表示注册状态下低电平持续时间；
 *  \return 0：成功 -1：失败
 *  
 */
int opencpu_get_cmsysctrl(int *sys_status, int *wakeup_out, int *net_light_time_ms, int *net_light_regist_time_ms, int *net_slake_registing_time_ms, int *net_slake_regist_time_ms);
/**
 *  \brief Brief description
 *  
 *  \param [in] sntp_ser NTP服务器域名
 *  \param [in] sntp_port NTP服务器端口号
 *  \param [in] set_rtc 是否同步更新RTC，0：不更新，1：更新
 *  \param [in] timeout 超时时间
 *  \param [in] user_callback 返回结果的回调函数
 *  \return 0：成功 -1：失败
 *  
 *  \details More details
 */
int opencpu_cmntp(unsigned char *sntp_ser,unsigned int sntp_port,unsigned int set_rtc,unsigned int timeout,oc_sntp_callback_t user_callback);
/**
 *  \brief 模组在浅睡眠或深度睡眠时可以被WAKEUP_IN引脚的低跳变沿唤醒。此函数用来设置被唤醒后的保持时间
 *  
 *  \param [in] lock_time 唤醒保持时间，1到300s
 *  \return Return 0：成功 -1：失败
 *  
 *  \details More details
 */
int opencpu_waketime_set(int lock_time);
/**
 *  \brief 模组在浅睡眠或深度睡眠时可以被WAKEUP_IN引脚的低跳变沿唤醒,然后等待一段时间后恢复睡眠，使用此命令可让模组立即进入睡眠
 *  
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_entersleep_mode(void);
/**
 *  \brief 获取SDK在首次下载到模组时的启动方式，该函数为SDK内部调用的回调函数，由用户填写返回值
 *  
 *  \return 1：首次下载到模组时以AT模式运行，须执行AT+ATCLOSE并重启，之后模组以opencpu方式运行。0：任何时候以opencpu模式运行
 *  
 *  \details 请联系技术支持人员确认所需启动模式
 */
int get_factory_mode(void);
/**
 *  \brief 获取模组当前的运行模式
 *  
 *  \return 0：AT模式 1：opencpu模式
 *  
 *  \details 
 */
int get_run_mode(void);
/**
 *  \brief 获取模组支持的硬件版本号
 *  
 *  \param [in] base_ver 保存版本号字符串的指针
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_get_base_version(unsigned char *base_ver);
/**
 *  \brief 获取当前固件支持的硬件版本
 *  
 *  \return 0：CM 5：LV
 *  
 *  \details 请一定确认固件和实际的硬件版本一致。后续可能会有更多硬件版本
 */
int get_band_version(void);
/**
 *  \brief 查询网络是否注册成功
 *  
 *  \return 1：已注册 0：未注册
 *  
 *  示例：下面代码用来循环等待模组驻网成功，然后执行其他数据收发操作
 *  @code 
 *  while(opencpu_cgact()!=1)
	{
		vTaskDelay(10);
	}
 *  @endcode
 */
int opencpu_cgact(void);

//获取信号质量
 /**
 *  \brief 查询当前信号质量
 *  
 *  \param [out] rssi 信号强度
 *  \param [out] rxqual 信号质量
 *  \return 空
 *  
 */
 void opencpu_csq(int *rssi,int*rxqual);
 


 /**
 *  \brief 执行系统重启
 *  
 *  \return 空
 *  
 *  
 */
 void opencpu_reboot(void);
 /**
 *  \brief 将模组IMEI备份，并将产品IMEI写入模组替代原IMEI，执行此函数之前必须先执行AT+CFUN=0
 *  
 *  \param [in] imei_t 要写入的产品imei字符串
 *  \return 0：成功 -1：失败
 *  
 *  \details IMEI写入有失败风险，请读出进行检验，确保写入成功
 */
 int opencpu_write_prod_imei(unsigned char* imei_t);
 /**
 *  \brief 读取备份区IMEI
 *  
 *  \param [in] imei 保存IMEI的数组地址
 *  \return 0：成功 -1：失败
 *  
 *  \details More details
 */
 int opencpu_read_bak_imei(unsigned char *imei);
/**
 *  \brief 获取SIM ICCID
 *  
 *  \param [out] buf 保存ICCID的地址
 *  \return 0：获取成功 -1：获取失败
 *  
 * 
 */
int  opencpu_iccid(unsigned char *buf);


 
 /**
 *  \brief 获取设备IMEI
 *  
 *  \param [out] pimei 保存IMEI的地址
 *  \return Return  0：成功 -1：失败
 *  
 *  示例：获取设备IMEI
 *  @code
 *  unsigned char local_imei[40] = {0};
 *  memset(local_imei,0,40);
	opencpu_get_imei(local_imei)
	opencpu_printf("IMEI:%s\n",local_imei);
 *  @endcode
 */
 int opencpu_get_imei(unsigned char *pimei);
 
/**
 *  \brief 获取设备IMSI
 *  
 *  \param [out] pimsi 保存IMSI的地址
 *  \return 0：成功 -1：失败
 *  
 *  示例：获取设备IMSI
 *  @code
 *  unsigned char local_imsi[40] = {0};
 *  memset(local_imsi,0,40);
	opencpu_printf("result:%d\n",opencpu_get_imsi(local_imsi));
	opencpu_printf("IMSI:%s\n",local_imsi);
 *  @endcode
 */
int opencpu_get_imsi(unsigned char *pimsi);

/**
 *  \brief 关闭模组浅睡眠，执行了此函数之后，设备将无法进入浅睡眠以及PSM等各种省电模式
 *  
 *  \return 空
 *  
 *  \details 该函数用于用户要执行耗时操作的时候，为防止模组意外进入省电模式，导致数据丢失、程序失控等，先执行此函数关闭睡眠，耗时操作执行完毕之后再开启
 */
void opencpu_lock_light_sleep(void);

/**
 *  \brief 开启浅睡眠，与关闭浅睡眠函数成对使用，开启之后，设备就会按照正常流程，在该进入省电模式的时候进入省电模式
 *  
 *  \return 空
 *  
 */
void opencpu_unlock_light_sleep(void);

/**
 *  \brief 获取PSM参数
 *  
 *  \param [out] req 保存PSM参数的结构体指针
 *  \return 0：成功 -1：失败
 *  
 *  \details 这个函数只能查到当前的设置值，这个设置值跟实际基站允许的值不一定相同，要获取准确的PSM参数，得用CEREG命令来查询
 *  示例：获取当前的PSM设置值,结构体的具体意义，请参考ril.h
 *  @code
 *  ril_power_saving_mode_setting_rsp_t psm_rsp1;
	opencpu_get_psmparam(&psm_rsp1);
 *  @endcode
 */
int opencpu_get_psmparam(ril_power_saving_mode_setting_req_t *req);



 /**
 *  \brief 设置PSM参数
 *  
 *  \param [in] req 保存设置PSM参数的结构体指针
 *  \return 0：成功 -1：失败
 *  
 *  \details 此函数执行之后，模组会向基站发出消息，进行PSM参数的设置，基站可能同意，可能不同意，根据网络状况稍作延时后，最终结果以CEREG命令查询到的为准
 *  示例：设置PSM参数
 *  @code
 *  ril_power_saving_mode_setting_req_t psm_req1;
	psm_req1.mode=1;
	psm_req1.req_prdc_rau=NULL;
	psm_req1.req_gprs_rdy_tmr=NULL;
	psm_req1.req_prdc_tau="00101011";
	psm_req1.req_act_time="00100100";
	 
	opencpu_set_psmparam(&psm_req1);
 *  @endcode
 */
 int opencpu_set_psmparam(ril_power_saving_mode_setting_req_t *req);
 /**
 *  \brief 设置EDRX
 *  
 *  \param [in] eDRX_mode EDRX模式，0或1，代表关闭或使能
 *  \param [in] act_type  ACT类别，只能为5
 *  \param [in] requested_eDRX_value EDRX时间参数
 *  \return 0成功，-1失败
 *  
 *  \details More details
 */
 int opencpu_set_edrx(int eDRX_mode,int act_type,char *requested_eDRX_value);
 /**
 *  \brief 读取EDRX设置
 *  
 *  \param [in] act_type ACT类别，只能为5
 *  \param [in] edrx_value EDRX时间参数
 *  \return 0成功，-1失败
 *  
 *  \details More details
 */
 int opencpu_read_edrx(int * act_type,unsigned char *edrx_value);
 /**
 *  \brief 读取EDRX生效值
 *  
 *  \param [in] edrxrdp_value EDRX生效参数的结构体，请参见ril.h
 *  \return 0成功，-1失败
 *  
 *  \details More details
 */
int opencpu_read_edrxrdp(ril_read_eDRX_dynamic_parameters_rsp_t *edrxrdp_value);
 /**
 *  \brief cfun的读取命令
 *  
 *  \param [out] cfun 当前CFUN的值
 *  \return 空 
 */
void opencpu_cfun_read(int * cfun);
/**
 *  \brief cfun的设置命令
 *  
 *  \param [in] cfun 要设置的cfun值
 *  \return 空
 *  
 */
void opencpu_cfun_excute(int cfun);

/**
 *  \brief CEREG读取
 *  
 *  \param [out] cereg_temp 保存CEREG返回参数的地址
 *  \return 空
 *  
 *  \details AT命令的使用方法请查阅AT文档，结构体的定义请参考ril.h
 */
void opencpu_cereg_read(ril_eps_network_registration_status_rsp_t * cereg_temp);
/**
 *  \brief CEREG设置
 *  
 *  \param [in] cereg 要设置的CEREG值
 *  \return 空
 *  
 *  \details 使用方法请查询AT文档
 */
void opencpu_cereg_excute(int cereg);

/**
 *  \brief wakein 引脚下降沿中断回调函数
 *  
 *  \return 空
 *  
 *  \details 该函数必须由用户实现，否则无法编译通过，该函数在wakein引脚出现下降沿时调用，该函数为中断回调函数，在该函数中不能执行耗时操作
 */
void opencpu_wakeup_callback(void);

/**
 *  \brief 判断模组是否是从PSM模式唤醒
 *  
 *  \return 1：PSM模式唤醒  0：关机状态下开机或者复位键复位
 *  

 */
int opencpu_is_boot_from_sleep(void);

  /** @} */ 
  /**

 * @defgroup DM_FUNCTIONS
 * 
   多形态终端上报接口，请参照SDK中示例文件使用
 *@{
 */

 /**
 *  \brief 设置DM上报的模式
 *  
 *  \param [in] auto_update true:周期性自动上报，false：手动单次上报
 *  \return 空
 *  
 *  \details More details
 */
void opencpu_dm_set_mode(bool auto_update);

 /**
 *  \brief DM启动
 *  
 *  \param [in] config DM参数配置
 *  \return true：成功 false:失败
 *  
 *  \details More details
 */
 bool opencpu_dm_start(dm_config_t config);
 /**
 *  \brief DM停止
 *  
 *  \return true：成功 false:失败
 *  
 *  \details More details
 */
 bool opencpu_dm_stop(void);
 /**
 *  \brief 发起一次DM上报
 *  
 *  \return Return true:成功 false:失败
 *  
 *  \details 此函数只在手动模式下调用有效
 */
 bool opencpu_dm_update(void);
 
   /** @} */ 
/**

 * @defgroup FOTA_FUNCTIONS 
 * 
   M5311目前支持基于onenet平台的差分固件升级，平台操作方法请参考OneNET平台发布的FOTA用户使用说明中"提供差分能力的升级系统界面"，部分相关文档请在OneNET平台上获取。
   FOTA操作方法请参见FOTA相关手册
 *@{
 */
 
/**
 *  \brief 发起升级包下载
 *  
 *  \return 空
 *  
 *  \details 尝试从平台上查询升级包并下载
 */
void opencpu_fota_try_download(void);

/**
 *  \brief 触发模组升级
 *  
 *  \return 空
 *  
 *  \details 下载完成后，即可触发升级，升级会自动关闭所有应用程序，进入升级流程，升级完成后自动重启
 */
void opencpu_fota_update(void);
/**
 *  \brief 设置FOTA上报流程的参数，重启生效
 *  
 *  \param [in] first 开机后第一次上报的起始时间，默认40，最小10，单位s
 *  \param [in] period 第一次上报失败的情况下，重试的间隔周期，默认120，最小120，单位s
 *  \param [in] count 上报失败的情况下，最多重试多少次，默认10，最小1，单位次
 *  \return Return 0：设置成功  -1：设置失败
 *  
 *  \details More details
 */
int opencpu_fota_set_report_param(int first,int period,int count);
/**
 *  \brief 获取当前FOTA上报流程的参数
 *  
 *  \param [in] first 开机后第一次上报的起始时间，单位s
 *  \param [in] period 第一次上报失败的情况下，重试的间隔周期，单位s
 *  \param [in] count 上报失败的情况下，最多重试多少次，单位次
 *  \return 空
 *  
 *  \details More details
 */
void opencpu_fota_get_report_param(int * first,int * period,int *count);
/**
 *  \brief FOTA事件回调函数，当有FOTA事件发生时，SDK自动调用此函数返回结果
 *  
 *  \param [in] event 事件主类型
 *  \param [in] state 事件子类型
 *  \return 空
 *  
 *  \details 该函数在SDK中调用，实现在5311_opencpu.c中，不可删除，用户可自行改写内容
 */
void opencpu_fota_event_cb(int event,int state);
/**
 *  \brief FOTA下载进度回调函数
 *  
 *  \param [in] current 当前下载的字节数
 *  \param [in] total 差分包的总字节数
 *  \return 空
 *  
 *  \details 该函数在SDK中调用，实现在5311_opencpu.c中，不可删除，用户可自行改写内容
 */
void opencpu_fota_progress_cb(int current,int total);
/**
 *  \brief 用户自定义程序版本号
 *  
 *  \return 版本号的字符串指针
 *  
 *  \details More details
 */
unsigned long opencpu_fota_version_cb(void);
/** @} */ 
  /**

 * @defgroup UART_FUNCTIONS  
 * 
   本模组一共开放四路UART,接收缓冲区为1024字节，超出将造成数据丢失。发送缓冲区为256字节。UART代码相关的结构体请参考hal_uart.h
 *@{
 */
 /**
 *  \brief 串口初始化函数
 *  
 *  \param [in] uart_port 要初始化的串口号
 *  \param [in] baud 波特率
 *  \param [in] user_cb 数据接收回调函数
 *  \return 空
 *  
 *  \details 此函数为简化的初始化函数，一部分默认参数固定成了默认参数，如果用户需要设置这些参数，可使用另一个串口初始化函数\n
 *  示例：初始化串口同时设置接收回调函数，实现串口收到什么数据就向发送方返回什么数据
 *  @code
 * 
 *  static void user_uart_callback(hal_uart_callback_event_t status, void *user_data)
   {
   char buffer[64];
   char *pbuf;
   pbuf = buffer;
   int temp1 = 1;  
   if(status == HAL_UART_EVENT_READY_TO_READ)
   {
	   while(temp1 > 0 )
	   {
		   memset(pbuf,0,64);
		   temp1 = opencpu_uart_receive(OPENCPU_MAIN_UART, pbuf, 64);
	       opencpu_uart_send(HAL_UART_1, pbuf, temp1);
	       temp1 = opencpu_uart_get_available_receive_bytes(HAL_UART_1);
	   }
	   
   } 
  }
   opencpu_uart_open(HAL_UART_1,HAL_UART_BAUDRATE_115200,user_uart_callback);
 *  @endcode
 */
 void opencpu_uart_open(hal_uart_port_t uart_port,hal_uart_baudrate_t baud,hal_uart_callback_t user_cb);
 /**
 *  \brief 串口初始化函数
 *  
 *  \param [in] uart_port 要初始化的串口号
 *  \param [in] l_config 详细的UART配置结构体指针
 *  \param [in] user_cb 数据接收回调函数
 *  \return 空
 *  
 *  \details 用户可以使用此初始化函数对UART的参数进行详细的设置\n
 *  示例：使用此函数初始化串口
 *  @code
 *  hal_uart_config_t uart_config;
	uart_config.baudrate = HAL_UART_BAUDRATE_115200;
	uart_config.parity = HAL_UART_PARITY_NONE;
	uart_config.stop_bit = HAL_UART_STOP_BIT_1;
	uart_config.word_length = HAL_UART_WORD_LENGTH_8;
	opencpu_uart_open_custom(HAL_UART_1,&uart_config, user_uart_callback);
 *  @endcode
 */
 void opencpu_uart_open_custom(hal_uart_port_t uart_port,hal_uart_config_t *l_config,hal_uart_callback_t user_cb);
 /**
 *  \brief 从指定串口的缓冲区获取串口数据
 *  
 *  \param [in] uart_port 串口号
 *  \param [in] pbuf 保存数据的指针
 *  \param [in] len 欲接收的数据长度
 *  \return 接收到的数据长度
 */
 int opencpu_uart_receive(hal_uart_port_t uart_port,unsigned char * pbuf,unsigned int len);
 /**
 *  \brief 串口发送函数
 *  
 *  \param [in] uart_port 串口号
 *  \param [in] pbuf 数据指针
 *  \param [in] len 要发送的数据长度
 *  \return 发送成功的字节数
 
 */
int opencpu_uart_send(hal_uart_port_t uart_port,unsigned char * pbuf,unsigned int len);
/**
 *  \brief 获取当前串口缓冲区接收到的数据长度
 *  
 *  \param [in] uart_port 串口号
 *  \return 缓冲区收到的数据长度
 *  

 */
int opencpu_uart_get_available_receive_bytes(hal_uart_port_t uart_port);
/**
 *  \brief 串口关闭函数
 *  
 *  \param [in] uart_port 串口号
 *  \return 空
 *  
 */
void  opencpu_uart_close(hal_uart_port_t uart_port);
 
  /** @} */ 
  
  
   /**

 * @defgroup NETWORK_FUNCTIONS  
 * 网络接口，TCP,UDP 收发相关函数及流程符合POSIX 标准，用户可以参照以下示例进行开发\n
   <pre>
   @code
   TCP测试：
   void tcp_test()
   {
	struct sockaddr_in server_addr;  
    int sock_fd;  
    int data_len;
	//填入服务器地址、端口
	inet_aton("47.93.217.230", &test_remote_addr);  
    test_remote_port = 2019;
	//创建TCP类型socket
	sock_fd = socket(AF_INET, SOCK_STREAM, 0);  
	if (sock_fd == -1) {  
        opencpu_printf ( "socket create error\n");
		 return;
    }  
	memset(&server_addr, 0, sizeof(server_addr));
	填充服务器地址信息结构体
	server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = test_remote_addr.s_addr;
    server_addr.sin_port = htons(test_remote_port);
	//连接到TCP服务器
    if(connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)))
	{
		opencpu_printf ( "tcp connect error\n");
		return;
	}
	    opencpu_printf ( "tcp data sending\n");
		//发送数据到服务器
	    send(sock_fd, (char *)test_msg, sizeof(test_msg), 0);
	    memset(test_rx_buf,0,100);
		//下面这句已被注释，用于非阻塞方式接收数据，无数据可收则立即返回
		//data_len = recvfrom(sock_fd, test_rx_buf,
                  //  100, MSG_TRUNC | MSG_DONTWAIT, (struct sockaddr*)&from, &fromlen);
        opencpu_printf ( "tcp waiting...\n");
		//阻塞方式等待服务器返回数据
        data_len = recv(sock_fd, test_rx_buf,100, MSG_TRUNC);
        opencpu_printf ( "waiting end\n");
        //打印出接收到的数据信息     
	    if(data_len >0)
		{
		    opencpu_printf ("get:%s\n",test_rx_buf);
			opencpu_printf ( "len:%d\n",data_len);			 
		}
		opencpu_printf ( "tcp waiting..1.\n");
		//再次阻塞等待服务器数据
        data_len = recv(sock_fd, test_rx_buf,100, MSG_TRUNC);
        opencpu_printf ( "waiting end1\n");
		//打印出接收到的数据信息
		if(data_len >0)
		{
		    opencpu_printf ("get:%s\n",test_rx_buf);
			opencpu_printf ( "len:%d\n",data_len);			 
		}

	close(sock_fd);
	
    }
	UDP测试：
	void udp_test()
{
    struct sockaddr_in server_addr;
    int sock_fd; 
    struct sockaddr_in from;
    int data_len;
    struct in_addr test_remote_addr;
uint16_t test_remote_port;
//填入要发送数据的IP地址
	inet_aton("47.93.217.230", &test_remote_addr);  
	//要发送数据的端口
    test_remote_port = 2019;	
	socklen_t fromlen = sizeof(struct sockaddr_in);
	//创建UDP类型的socket
	sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sock_fd == -1)
	{
		opencpu_printf ( "socket create error\n");
		return;
	}
	memset(&server_addr, 0, sizeof(server_addr)); 
	//填充地址结构体
	server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = test_remote_addr.s_addr;  
    server_addr.sin_port = htons(test_remote_port);  
   //UDP中该连接函数只完成一些本地操作，传入一些参数
	connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)); 
	opencpu_printf ("data sending\n");
	//UDP发送数据
	send(sock_fd, (char *)test_msg, sizeof(test_msg), 0);
	 memset(test_rx_buf,0,100);
    //这一句被注释掉了，这是非阻塞方式接受数据，如果没有数据可收，则立即返回
	//data_len = recvfrom(sock_fd, test_rx_buf,
                   // 100, MSG_TRUNC | MSG_DONTWAIT, (struct sockaddr*)&from, &fromlen);
    opencpu_printf ( "waiting...\n");
    //这是阻塞方式接受数据，会阻塞等到数据到来才返回
    data_len = recvfrom(sock_fd, test_rx_buf,
                    100, MSG_TRUNC, (struct sockaddr*)&from, &fromlen);
    opencpu_printf("waiting end\n");   
	//在串口打印出服务器返回的数据信息                
	if(data_len >0)
		{
		    opencpu_printf ( "get:%s\n",test_rx_buf);
			opencpu_printf ( "len:%d\n",data_len);			 
		}
	//关闭socket
	close(sock_fd);	
   }
  @endcode
  </pre>

 *@{
 */
  /**
 *  \brief 执行PING功能
 *  
 *  \param [in] remote remote IP
 *  \param [in] count 要发送的包个数
 *  \param [in] pktsz 每个包的大小
 *  \param [in] callback 执行结果的回调函数
 *  \return 空
 *  
 *  示例：下面测试180.97.33.108的可达性，发三个包，包携带64个字节的数据
 *  @code
 *  
 *  opencpu_ping("180.97.33.108",3,64,test_ping_cb);
 *  void test_ping_cb(ping_result_type_t type, void* p)
{
    switch(type)
	{
        case PING_TOTAL_RESULT: 
		{
		    if (p) {
				ping_result_t* ping_result = (ping_result_t*)p;
		       opencpu_printf ("\r\n--- %s ping statistics ---\r\n"
		            "%d packets transmitted, %d received, %d%% packet loss\r\n"
		            "rtt min/avg/max = %d/%d/%d", inet_ntoa(ping_result->ping_target),
		            (int)ping_result->total_num, (int)ping_result->recv_num,  (int)((ping_result->lost_num * 100)/ping_result->total_num),
		            ping_result->min_time,ping_result->avg_time,ping_result->max_time);

		    } else {
		        break;//response.pdata = "+Ping: finish, no result!";
		    }
		}
		break;
		case PING_PACKET_RESULT:
		{
			ping_packet_result_t* ping_result = (ping_packet_result_t*)p;
			if (p == NULL)
			{
                break;
            }
			else if (ping_result->is_timeout == true)
			{
                opencpu_printf ("+PINGERR: 1");
				break;
            }
			else
			{
                if (ping_result->is_ipv4 == true)
				{
					opencpu_printf ("+PING: %d.%d.%d.%d,%d,%d",ping_result->ip_address[0],
                           ping_result->ip_address[1], ping_result->ip_address[2], ping_result->ip_address[3],
						   ping_result->ttl,ping_result->rtt);
                }
				else
				{
                   opencpu_printf ("+PING: %x:%x:%x:%x:%x:%x:%x:%x,%d,%d",ping_result->ip_address[0],
                         ping_result->ip_address[1],ping_result->ip_address[2], ping_result->ip_address[3],
                         ping_result->ip_address[4],ping_result->ip_address[5],
                         ping_result->ip_address[6],ping_result->ip_address[7],
						 ping_result->ttl,ping_result->rtt);
                }
            }
		}
		break;
	}
}

 *  @endcode
 */
 void opencpu_ping(unsigned char* remote,int count,int pktsz,ping_request_result_t callback);
/**
 *  \brief 执行域名解析功能
 *  
 *  \param [in] domain_name 要解析的域名字符串
 *  \param [in] server_ip 域名解析服务器IP地址
 *  \param [in] test_dns_cb 回调函数
 *  \return 0：成功 -1：失败
 *  
 *  示例：下面测试获取百度的IP地址，填写域名；第二个参数是DNS服务器IP，没有就填0；最后是回调函数
 *  @code 
 *  opencpu_get_host_by_name("www.baidu.com",0,test_cmdns_cb)
    @endcode
    回调函数的写法如下：
    @code
  void test_cmdns_cb(unsigned char *ip)
{
	
    opencpu_printf ("opencpu dns:%s\n",ip);	
}

 *  @endcode
 */
int  opencpu_get_host_by_name(unsigned char *domain_name,unsigned char *server_ip,dns_request_result_t test_dns_cb);
 
  /** @} */ 
  
   /**

 * @defgroup FLASH_FUNCTIONS  
   支持 128K Flash 空间，地址从 0 开始。注意，擦除函数会擦除指定地址区域所涉及的 flash 块，以 4K 字节为单位。
 *@{
 */
 
 /**
 *  \brief flash读数据函数
 *  
 *  \param [in] addr flash地址
 *  \param [in] data 保存读出数据的地址
 *  \param [in] len 要读取的长度
 *  \return 0：成功 -1：失败
 *  
 */
int opencpu_flash_read(uint32_t addr, unsigned char * data,uint32_t len);
/**
 *  \brief flash擦除函数
 *  
 *  \param [in] addr 要擦除的起始地址，此地址必须4K字节对齐，否则将返回错误
 *  \param [in] len 要擦除的长度
 *  \return 0：成功 -1：失败
 *  
 *  \details 我们提供了128K flash供用户使用，地址从0到128K，flash以4K为一个块。flash只能以块为最小单位进行擦除，擦除函数会根据传入的起始地址和长度，自动计算要擦除哪些块。举个例子：
当要擦除从地址2048开始的400字节时，我们调用：\n
		opencpu_flash_erase(2048,400);\n
此时这个要擦除的地址范围经过计算，处于0到4095这个块内，则整个0到4095地址范围的flash都会被擦除。
当要擦除从地址2048开始的4096字节时，我们调用：\n
		opencpu_flash_erase(2048,4096);\n
此时这个要擦除的地址范围经过计算，处于0到4095，4096到8192这两个块内，则整个0到4095，4096到8192这两个块都会被擦除。

 */
int opencpu_flash_erase(uint32_t addr, uint32_t len);
/**
 *  \brief flash写入
 *  
 *  \param [in] addr 要写入的地址
 *  \param [in] pdata 要写入的数据指针
 *  \param [in] len 要写入的长度
 *  \return 0：成功 -1：失败
 *  
 *  \details 单独修改flash中某一个字节，而不影响其他位置数据的方法：比如，要往地址2048写一个字节0xEE，则应先读出这个字节地址所在flash块的内容到数组中，在数组中修改这个字节地址的值为0xEE，然后擦除这一块flash，把修改后的数组内容写回到这个块。代码示例如下：
 *  @code
	unsinged char buffer[4096];
	memset(buffer,0,4096);
	opencpu_flash_read(0,buffer,4096);
	buffer[2048]=0xEE;
	//这个擦除函数会擦除地址0到4095这个flash块
	opencpu_flash_erase(2048,1);
	opencpu_flash_write(0,buffer,4096);
	@endcode

 */
int opencpu_flash_write(unsigned int addr,unsigned char *pdata,int len);
 
  /** @} */ 
   /**

 * @defgroup RTC_FUNCTIONS  
 *  注意，RTC定时器能把模组从PSM状态下唤醒
 *@{
 */
 
 /**
 *  \brief 设置RTC时间
 *  
 *  \param [in] time_string 时间字符串
 *  \return 0：成功 -1：失败
 *  
 *  示例：设置时间及相应时区，字符串格式为"yy/MM/dd,hh:mm:ss+-zz"
 *  @code 
 *  opencpu_rtc_set_time("94/05/06,22:10:00+08");
 *  @endcode 
 */
 int opencpu_rtc_set_time(unsigned char *time_string);
 
/**
 *  \brief 获取RTC时间
 *  
 *  \param [out] time_string 保存时间字符串的地址
 *  \return Return 空
 *  
 *  \details 驻网成功后，模组会从基站获取到当前的时间和相应的时区，然后自动设置到RTC中
 *  示例：获取当前RTC时间
 *  @code
 *  unsigned char time_string[50];
    memset(time_string,0,50);
	opencpu_rtc_get_time(time_string);

 *  @endcode
 */
 void opencpu_rtc_get_time(unsigned char *time_string);
/**
 *  \brief RTC timer创建
 *  
 *  \param [out] handle  保存timer句柄的指针
 *  \param [in] time_out 定时时间，单位是100ms
 *  \param [in] is_periodic 是否到期后自动重新开始
 *  \param [in] callback 到期回调函数
 *  \return 0：成功 小于0：失败
 *  
 */
int opencpu_rtc_timer_create(uint32_t *handle,const uint32_t time_out, const bool is_periodic,rtc_sw_timer_callback_t callback);
/**
 *  \brief 启动定时器
 *  
 *  \param [in] handle 定时器句柄
 *  \return 0：成功 小于0：失败
 *  
 */
int opencpu_rtc_timer_start(uint32_t handle);
/**
 *  \brief 停止定时器
 *  
 *  \param [in] handle 定时器句柄
 *  \return 0：成功 小于0：失败
 *  
 */
int opencpu_rtc_timer_stop(uint32_t handle);
/**
 *  \brief 删除定时器
 *  
 *  \param [in] handle 定时器句柄
 *  \return 0：成功 小于 0：失败
 *  
 *  \details 0：成功 小于0：失败
 */
int opencpu_rtc_timer_delete(uint32_t handle);
 
  /** @} */ 
  
   /**

 * @defgroup ADC_FUNCTIONS  
 *  模组一共提供四路ADC，精度为10位，直接使用即可，不需初始化配置等过程。
 *@{
 */
/**
 *  \brief 
 *  
 *  \param [in] chan ADC通道号，支持HAL_ADC_CHANNEL_0，HAL_ADC_CHANNEL_1，HAL_ADC_CHANNEL_3，HAL_ADC_CHANNEL_4
 *  \return 电压值
 *  
 */
int opencpu_adc(hal_adc_channel_t chan);
 
  /** @} */ 
  
   /**

 * @defgroup PWM_FUNCTIONS  
 * 模组支持两路PWM，0和3，具体引脚请参考资源综述文档
 *@{
 */

/**
 *  \brief 初始化PWM
 *  
 *  \param [in] channel 通道号，0或3
 *  \param [in] freq 频率
 *  \param [in] ratio 占空比
 *  \return Return 空
 *  
 *  示例：
 *  @code
 *  //设置占空比，0到100
   #define DUTY_RATIO  50
   //设置频率   
   #define PWM_FREQ   2000
   //初始化PWM
   opencpu_pwm_init(3,PWM_FREQ,DUTY_RATIO);
   //开始PWM输出
   opencpu_pwm_start(3);
   vTaskDelay(1000);
   //停止PWM输出
   opencpu_pwm_stop(3);	
   //释放PWM
   opencpu_pwm_deinit(3);

 *  @endcode
 *  \details More details
 */
void opencpu_pwm_init(unsigned int channel,unsigned int freq,unsigned int ratio);
 /**
 *  \brief 启动PWM输出
 *  
 *  \param [in] channel 通道号，0或3
 *  \return Return 空
 *  
 *  \details More details
 */
 void opencpu_pwm_start(unsigned int channel);
/**
 *  \brief 停止PWM输出
 *  
 *  \param [in] channel 通道号，0或3
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_pwm_stop(unsigned int channel);
/**
 *  \brief PWM设置占空比
 *  
 *  \param [in] channel PWM通道，0或3
 *  \param [in] ratio 占空比
 *  \return 空
 *  
 *  \details More details
 */
void opencpu_pwm_set_duty_cycle(unsigned int channel,unsigned int ratio);
/**
 *  \brief PWM设置频率
 *  
 *  \param [in] channel PWM通道，0或3
 *  \param [in] freq 频率
 *  \return 空
 *  
 *  \details More details
 */
void opencpu_pwm_set_frequency(unsigned int channel,unsigned int freq);
 /**
 *  \brief 关闭PWM
 *  
 *  \param [in] channel 通道号，0或3
 *  \return Return 空
 *  
 *  \details More details
 */
 void opencpu_pwm_deinit(unsigned int channel);
  /** @} */ 
  
   /**

 * @defgroup SPI_FUNCTIONS  
 * 本模组支持一路SPI Master，CS引脚由软件自动控制。SPI的结构体和参数说明可以参见spi.h
 *@{
 */
/**
 *  \brief 初始化SPI
 *  
 *  \param [in] spi_config 配置SPI的结构体指针
 *  \return 0：成功 小于 0：失败
 *  
 *  示例：
 *  @code
 *  unsigned char spi_rx_buf[2]= {0};
    unsigned char spi_tx_buf[2]= {0};
    unsigned char spi_tx = 5;

    hal_spi_master_config_t l_config;	
    hal_spi_master_send_and_receive_config_t spi_send_and_receive_config;
    //配置SPI参数
    l_config.bit_order = HAL_SPI_MASTER_MSB_FIRST;
    l_config.slave_port = HAL_SPI_MASTER_SLAVE_0;
    l_config.clock_frequency = 1000000;
    l_config.phase = HAL_SPI_MASTER_CLOCK_PHASE1;
    l_config.polarity = HAL_SPI_MASTER_CLOCK_POLARITY0;
    //初始化SPI
    opencpu_spi_init(&l_config);
    //spi发送一个字节
    opencpu_spi_write(&spi_tx,1);
    spi_tx_buf[0]= 0xEE;
    spi_tx_buf[1] = 0x25;
    //发送一个字节，读取两个字节，使用这个函数，读取长度必须大于发送长度，否则会直接报错
    opencpu_spi_rw(spi_tx_buf,1,spi_rx_buf,2);

    使用完后，调用释放函数：
    opencpu_spi_deinit();
    如果用户不需要硬件自动控制CS，把CS引脚初始化为GPIO功能即可

 *  @endcode
 */
int opencpu_spi_init(hal_spi_master_config_t *spi_config);
/**
 *  \brief 关闭SPI
 *  \return 空
 *  
 */
void opencpu_spi_deinit(void);
/**
 *  \brief SPI读写，读取长度必须大于发送长度
 *  
 *  \param [in] send_buf 要发送的数据指针
 *  \param [in] send_len 要发送的数据长度
 *  \param [in] read_buf 保存读取数据的地址
 *  \param [in] read_len 要读取的数据长度
 *  \return 0：成功 小于0：失败
 *  
 */
int opencpu_spi_rw(unsigned char *send_buf,int send_len,unsigned char *read_buf,int read_len);
/**
 *  \brief SPI发送
 *  
 *  \param [in] send_buf 要发送的数据指针
 *  \param [in] len 要发送的长度，最大32
 *  \return 0：成功 小于0：失败
 *  
 */
int  opencpu_spi_write(unsigned char *send_buf,int len);
/**
 *  \brief spi读取
 *  
 *  \param [in] buf 保存读取数据的内存指针
 *  \param [in] len 要读取的长度，最大32
 *  \return 0：成功 小于0：失败
 *  
 *  \details More details
 */
int opencpu_spi_read(unsigned char *buf,int len);
  /** @} */ 
   /**

 * @defgroup I2C_FUNCTIONS  
 * 本模组支持一路I2C
 *@{
 */
/**
 *  \brief I2C初始化
 *  
 *  \return 空 \n
 *  示例：使用I2C接口进行数据读写
 *  @code
 *  opencpu_i2c_init();
 *  unsigned char buffer[2];
	//向总线地址为0x77的外设写入0xD0，然后读出一个字节
	opencpu_i2c_write_read(0x77,0xD0, buffer, 1);
	//从总线地址为0x77的外设读出2个字节
	opencpu_i2c_read(0x77,buffer, 2);
	//向总线地址为0x77的外设写入2个字节
	opencpu_i2c_write(0x77,buffer, 2);
    //使用完毕后，调用I2C释放函数：
	opencpu_i2c_deinit();

 *  @endcode
 */
void opencpu_i2c_init(void);
/**
 *  \brief 设置I2C速率
 *  
 *  \param [in] freq 速率
 *  \return Return 空
 *  
 *  \details I2C设备初始化之后调用即可
 */
void opencpu_i2c_set_freq(hal_i2c_frequency_t freq);
/**
 *  \brief 获取I2C速率
 *  
 *  \return Return 当前速率
 *  
 *  \details 默认速率为50kbps
 */
hal_i2c_frequency_t  opencpu_i2c_get_freq(void);

/**
 *  \brief i2c向一个器件发送一个字节，然后读取n个字节
 *  
 *  \param [in] slave_addr 外设的I2C地址
 *  \param [in] reg  要写入外设的一个字节
 *  \param [in] data 要保存读出数据的地址
 *  \param [in] len 要读取的数据长度
 *  \return 0：成功 -1：失败
 *  
 
 */
int opencpu_i2c_write_read(uint8_t slave_addr, uint8_t reg, uint8_t *data, uint16_t len);

/**
 *  \brief I2C读数据
 *  
 *  \param [in] slave_addr I2C外设地址
 *  \param [in] data 要读取的数据地址
 *  \param [in] len 要读取的数据长度
 *  \return 0：成功 -1：失败
 *  
 */
int opencpu_i2c_read(uint8_t slave_addr, uint8_t *data, uint16_t len);
/**
 *  \brief I2C写数据
 *  
 *  \param [in] slave_addr I2C外设地址
 *  \param [in] data 要发送的数据地址
 *  \param [in] len 要发送的数据长度
 *  \return 0：成功 -1：失败
 *  
 */
int opencpu_i2c_write(uint8_t slave_addr, uint8_t *data, uint16_t len);
/**
 *  \brief 关闭I2C
 *  
 *  \return 空
 *  
 */
void opencpu_i2c_deinit(void);
  /** @} */ 
  
   /**

 * @defgroup GPIO_FUNCTIONS  
 * M5311 模组只要没被复用成特殊功能的引脚，都可以当做普通GPIO来使用，模组引脚号、MT2625GPIO号、opencpu引脚功能名称三者的对应关系见附件Excel文件，相关结构体请参考hal_gpio.h，配置GPIO的接口函数如下所示。\n
  1.引脚初始化
  @code
  hal_gpio_status_t hal_gpio_init(hal_gpio_pin_t gpio_pin);
  @endcode
  2.引脚反初始化
  @code
  hal_gpio_status_t hal_gpio_deinit(hal_gpio_pin_t gpio_pin);
  @endcode
  3.设置引脚复用功能，引脚可配置的功能见hal_pinmux_define.h\
  @code
  hal_pinmux_status_t hal_pinmux_set_function(hal_gpio_pin_t gpio_pin, uint8_t function_index);
  @endcode
  4.获取引脚输入电平
  @code
  hal_gpio_status_t hal_gpio_get_input(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);
  @endcode
  5.设置引脚输出电平
  @code
  hal_gpio_status_t hal_gpio_set_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t gpio_data);
  @endcode
  6.获取引脚输出电平
  @code
  hal_gpio_status_t hal_gpio_get_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);
  @endcode
  7.设置引脚输入或输出
  @code
  hal_gpio_status_t hal_gpio_set_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t gpio_direction);
  @endcode
  8.获取当前引脚是输入还是输出
  @code
  hal_gpio_status_t hal_gpio_get_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t *gpio_direction);
  @endcode
  9.设置引脚为高阻状态
  @code
  hal_gpio_status_t hal_gpio_set_high_impedance(hal_gpio_pin_t gpio_pin);
  @endcode
  10.清除引脚高阻状态
  @code
  hal_gpio_status_t hal_gpio_clear_high_impedance(hal_gpio_pin_t gpio_pin);
  @endcode
  11.反转引脚输出状态
  @code
  hal_gpio_status_t hal_gpio_toggle_pin(hal_gpio_pin_t gpio_pin);
  @endcode
  12.设置引脚上拉电阻
  @code
  hal_gpio_status_t hal_gpio_pull_up(hal_gpio_pin_t gpio_pin);
  @endcode
  13.设置引脚上拉电阻
  @code
  hal_gpio_status_t hal_gpio_pull_down(hal_gpio_pin_t gpio_pin);
  @endcode
  14.关闭引脚上拉电阻或下拉电阻
  @code
  hal_gpio_status_t hal_gpio_disable_pull(hal_gpio_pin_t gpio_pin);
  @endcode
 
 *@{
 */
 
 
  /** @} */ 
  
    /**

 * @defgroup RTOS_INTRODUCTION 
 * M5311使用FreeRTOS V8.2.0嵌入式操作系统，所有的接口和标准FREERTOS一致，遇到相关问题，开发者可登录https://www.freertos.org/搜索相关资料解决，也可联系我们解决。相关结构体、参数等可以参考os目录下头文件，此处给出部分主要功能函数示例
 *  \n示例1：任务创建
  *<pre>
 BaseType_t xTaskCreate(
							  TaskFunction_t pvTaskCode,
							  const char * const pcName,
							  uint16_t usStackDepth,
							  void *pvParameters,
							  UBaseType_t uxPriority,
							  TaskHandle_t *pvCreatedTask
						  );</pre>
 *  \param 任务名
 *	\param 任务的堆栈大小(字数，本平台一个字为4字节)
 *	\param 私有参数指针
 *	\param 任务优先级（如无特殊要求，填TASK_PRIORITY_NORMAL即可）
 *	\param 任务句柄指针
    @return pdPASS或者其他错误码
  
   \n示例2：任务删除,参数为任务句柄，如果为NULL，则删除调用此函数的任务
   @code
   vTaskDelete( TaskHandle_t xTaskToDelete )
   @endcode
   示例3：任务内延时,延时单位为10ms
   @code
   vTaskDelay()
   @endcode
   示例4：内存分配释放
   @code
   void *pvPortMalloc( size_t xSize );
   void vPortFree( void *pv )
   @endcode
 *@{
 */
 /**
 *  \brief FreeRTOS栈溢出回调函数
 *  
 *  \param [in] pxTask 任务句柄指针
 *  \param [in] pcTaskName 任务名
 *  \return 空
 *  
 *  \details More details
 */
void opencpu_stack_overflow_hook(xTaskHandle *pxTask, signed portCHAR * pcTaskName);
/**
 *  \brief FreeRTOS时钟心跳回调函数，
 *  
 *  \return Return 空
 *  
 *  \details More details
 */
void vApplicationTickHook( void );
/**
 *  \brief FreeRTOS空闲任务回调函数
 *  
 *  \return Return 空
 *  
 *  \details More details
 */
void opencpu_task_idle_hook(void);

  /** @} */ 
 /** @} */  
