#ifndef __OPTION_H__
#define __OPTION_H__

#if __has_include("HAL_Conf.h")  /*在当前工程文件夹查找配置文件*/
# include "HAL_Conf.h"
#endif

#if __has_include("configs/HAL_Conf.h") /*在 variant/configs 文件夹查找配置文件*/
# include "configs/HAL_Conf.h"
#endif

#endif /*__OPTION_H__*/
