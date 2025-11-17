
#ifndef _MV_CAMERA_CTRL_H_
#define _MV_CAMERA_CTRL_H_

#include "MvErrorDefine.h"
#include "CameraParams.h"

/**
*  @brief  ��̬�⵼�뵼������
*/
#ifndef MV_CAMCTRL_API

    #if (defined (_WIN32) || defined(WIN64))
        #if defined(MV_CAMCTRL_EXPORTS)
            #define MV_CAMCTRL_API __declspec(dllexport)
        #else
            #define MV_CAMCTRL_API __declspec(dllimport)
        #endif
    #else
        #ifndef __stdcall
            #define __stdcall
        #endif

        #ifndef MV_CAMCTRL_API
            #define  MV_CAMCTRL_API
        #endif
    #endif

#endif

#ifndef IN
    #define IN
#endif

#ifndef OUT
    #define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif 

/************************************************************************/
/* ����Ļ���ָ��Ͳ���                                         */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_GetSDKVersion()
 *  @brief  ��ȡSDK�汾��
 *  @param  
 *  @return ʼ�շ���4�ֽڰ汾�� |��    |��    |����  |  ����|
                                 8bits  8bits  8bits  8bits 
 ************************************************************************/
MV_CAMCTRL_API unsigned int __stdcall MV_CC_GetSDKVersion();

/************************************************************************
 *  @fn     MV_CC_EnumerateTls()
 *  @brief  ��ȡ֧�ֵĴ����
 *  @return ֧�ֵĴ������ 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumerateTls();

/************************************************************************
 *  @fn     MV_CC_EnumDevices()
 *  @brief  ö���豸
 *  @param  nTLayerType            [IN]           ö�ٴ����
 *  @param  pstDevList             [OUT]          �豸�б�
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevices(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList);

/************************************************************************
 *  @fn     MV_CC_EnumDevicesEx()
 *  @brief  ���ݳ�������ö���豸
 *  @param  nTLayerType            [IN]           ö�ٴ����
 *  @param  pstDevList             [OUT]          �豸�б�
 *  @param  pManufacturerName      [IN]           ��������
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList, IN const char* pManufacturerName);

/************************************************************************
 *  @fn     MV_CC_IsDeviceAccessible()
 *  @brief  �豸�Ƿ�ɴ�
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @param  nAccessMode            [IN]           ����Ȩ��
 *  @return �ɴ����true�����ɴ����false 
 ************************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceAccessible(IN MV_CC_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode);

/************************************************************************
 *  @fn     MV_CC_CreateHandle()
 *  @brief  �����豸���
 *  @param  handle                 [OUT]          �����ַ
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandle(OUT void ** handle, IN const MV_CC_DEVICE_INFO* pstDevInfo);

/************************************************************************
 *  @fn     MV_CC_CreateHandleWithoutLog
 *  @brief  �����豸�������������־
 *  @param  handle                 [OUT]          �����ַ
 *  @param  pstDevInfo             [IN]           �豸��Ϣ�ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleWithoutLog(OUT void ** handle, IN const MV_CC_DEVICE_INFO* pstDevInfo);

/************************************************************************
 *  @fn     MV_CC_DestroyHandle()
 *  @brief  �����豸���
 *  @param  handle                 [IN]          ���
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyHandle(IN void * handle);

/************************************************************************
 *  @fn     MV_CC_OpenDevice()
 *  @brief  ���豸
 *  @param  handle                 [IN]          ���
 *  @param  nAccessMode            [IN]          ����Ȩ��
 *  @param  nSwitchoverKey         [IN]          �л�����Ȩ��ʱ����Կ
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
#ifdef __APPLE__
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode, IN unsigned short nSwitchoverKey);
#else

MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode = MV_ACCESS_Exclusive, IN unsigned short nSwitchoverKey = 0);
#endif

/***********************************************************************
 *  @fn         MV_CC_CloseDevice
 *  @brief      �ر����
 *  @param       handle                 [IN]          ���
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseDevice(IN void* handle);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallBackEx
 *  @brief      ע��ͼ�����ݻص���chunk
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx(void* handle, 
                                                         void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),
                                                         void* pUser);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallbackForRGB
 *  @brief      ע��ͼ�����ݻص���RGB
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForRGB(void* handle, 
                                                         void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),
                                                         void* pUser);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallbackForBGR
 *  @brief      ע��ͼ�����ݻص���BGR
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackForBGR(void* handle, 
                                                         void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser),
                                                         void* pUser);

/***********************************************************************
 *  @fn         MV_CC_StartGrabbing
 *  @brief      ��ʼȡ��
 *  @param       handle                 [IN]          ���
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartGrabbing(IN void* handle);

/***********************************************************************
 *  @fn         MV_CC_StopGrabbing
 *  @brief      ֹͣȡ��
 *  @param       handle                 [IN]          ���
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopGrabbing(IN void* handle);

/***********************************************************************
 *  @fn         MV_CC_GetImageForRGB
 *  @brief      ��ȡһ֡RGB���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForRGB(IN void* handle, IN OUT unsigned char * pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);

/***********************************************************************
 *  @fn         MV_CC_GetImageForBGR
 *  @brief      ��ȡһ֡BGR���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageForBGR(IN void* handle, IN OUT unsigned char * pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pFrameInfo, int nMsec);

/***********************************************************************
 *  @fn         MV_CC_GetOneFrameTimeout
 *  @brief      ���ó�ʱ���ƻ�ȡһ֡ͼƬ��SDK�ڲ��ȴ�ֱ��������ʱ���أ�
                �˽ӿڿ�������ȡ��ƽ���ԣ��ʺ����ڶ�ƽ����Ҫ��ϸߵĳ���
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @param       nMsec                  [IN]          �ȴ���ʱʱ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameTimeout(IN void* handle, IN OUT unsigned char * pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pFrameInfo, unsigned int nMsec);

/***********************************************************************
 *  @fn         MV_CC_Display
 *  @brief      ��ʾһ֡ͼ��ע����ʾ���ڣ��ڲ��Զ���ʾ
 *  @param       handle                 [IN]          ���
 *  @param       hWnd                   [IN]          ��ʾ���ھ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_Display(IN void* handle, void* hWnd);

/***********************************************************************
 *  @fn         MV_CC_SetImageNodeNum
 *  @brief      ����SDK�ڲ�ͼ�񻺴�ڵ��������Χ[1, 30]����ץͼǰ����
 *  @param       handle                 [IN]          ���
 *  @param       nNum                   [IN]          ����ڵ����
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetImageNodeNum(IN void* handle, unsigned int nNum);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(IN void* handle, IN OUT MV_IMAGE_BASIC_INFO* pstInfo);
 *  @brief  ��ȡͼ�������Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_IMAGE_BASIC_INFO* pstInfo     [IN][OUT]   ���ظ��������й����ͼ�������Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ο� CameraParam.h �е� MV_IMAGE_BASIC_INFO ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageInfo(IN void* handle, IN OUT MV_IMAGE_BASIC_INFO* pstInfo);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void* handle, IN OUT MV_CC_DEVICE_INFO* pstDevInfo);
 *  @brief  ��ȡ�豸��Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_CC_DEVICE_INFO* pstDevInfo    [IN][OUT]   ���ظ��������й�����豸��Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ο� CameraParam.h �е� MV_CC_DEVICE_INFO ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void * handle, IN OUT MV_CC_DEVICE_INFO* pstDevInfo);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle, IN OUT MV_ALL_MATCH_INFO* pstInfo);
 *  @brief  ��ȡ�������͵���Ϣ
 *  @param  void* handle                     [IN]        ������
 *  @param  MV_ALL_MATCH_INFO* pstInfo       [IN][OUT]   ���ظ��������й�����������͵���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ο� CameraParam.h �е� MV_ALL_MATCH_INFO ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle, IN OUT MV_ALL_MATCH_INFO* pstInfo);




/************************************************************************/
/* ���úͻ�ȡ������������ܽӿ�                                 */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           OUT MVCC_INTVALUE *pIntValue);
 *  @brief  ��ȡInteger����ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValue(IN void* handle,IN const char* strKey,OUT MVCC_INTVALUE *pIntValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,
                                                           IN const char* strKey,
                                                           IN unsigned int nValue);
 *  @brief  ����Integer������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ������Ϣ��Ϊ"Width"
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValue(IN void* handle,IN const char* strKey,IN unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,
                                                            IN const char* strKey,
                                                            OUT MVCC_ENUMVALUE *pEnumValue);
 *  @brief  ��ȡEnum����ֵ
 *  @param  void* handle                   [IN]        ������
 *  @param  char* strKey                   [IN]        ���Լ�ֵ�����ȡ���ظ�ʽ��Ϣ��Ϊ"PixelFormat"
 *  @param  MVCC_ENUMVALUE* pEnumValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,IN const char* strKey,OUT MVCC_ENUMVALUE *pEnumValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,
                                                            IN const char* strKey,
                                                            IN unsigned int nValue);
 *  @brief  ����Enum������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ�����ȡ���ظ�ʽ��Ϣ��Ϊ"PixelFormat"
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,IN const char* strKey,IN unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,
                                                             IN const char* strKey,
                                                             OUT MVCC_FLOATVALUE *pFloatValue);
 *  @brief  ��ȡFloat����ֵ
 *  @param  void* handle                     [IN]        ������
 *  @param  char* strKey                     [IN]        ���Լ�ֵ
 *  @param  MVCC_FLOATVALUE *pFloatValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,IN const char* strKey,OUT MVCC_FLOATVALUE *pFloatValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,
                                                             IN const char* strKey,
                                                             IN float fValue);
 *  @brief  ����Enum������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ
 *          float fValue                [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,IN const char* strKey,IN float fValue);
	
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,
                                                            IN const char* strKey,
                                                            OUT bool *pBoolValue);
 *  @brief  ��ȡBoolean����ֵ
 *  @param  void* handle                     [IN]        ������
 *  @param  char* strKey                     [IN]        ���Լ�ֵ
 *  @param  bool *pBoolValue                 [IN][OUT]   ���ظ��������й��������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,IN const char* strKey,OUT bool *pBoolValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,
                                                            IN const char* strKey,
                                                            IN bool bValue);
 *  @brief  ����Boolean������ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  char* strKey                [IN]        ���Լ�ֵ
 *          bool bValue                 [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,IN const char* strKey,IN bool bValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,
                                                              IN const char* strKey,
                                                              OUT MVCC_STRINGVALUE *pStringValue);
 *  @brief  ��ȡString����ֵ
 *  @param  void* handle                       [IN]        ������
 *  @param  char* strKey                       [IN]        ���Լ�ֵ
 *  @param  MVCC_STRINGVALUE *pStringValue     [IN][OUT]   ���ظ��������й�������Խṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,IN const char* strKey,OUT MVCC_STRINGVALUE *pStringValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,
                                                              IN const char* strKey,
                                                              IN const char * sValue);
 *  @brief  ����String������ֵ
 *  @param  void* handle                  [IN]        ������
 *  @param  char* strKey                  [IN]        ���Լ�ֵ
 *          char * sValue                 [IN]        ��Ҫ���õ����������ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,IN const char* strKey,IN const char * sValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,
                                                               IN const char* strKey);
 *  @brief  ����Command������ֵ
 *  @param  void* handle                  [IN]        ������
 *  @param  char* strKey                  [IN]        ���Լ�ֵ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,IN const char* strKey);




/************************************************************************/
/* ���������ȡ�����ã���ģ������нӿڣ����𲽷�������������������ܽӿڴ���   */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������ȵ���Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                  unsigned int    nCurValue;      // ���������ǰ�Ŀ���ֵ
 *                  unsigned int    nMax;           // ��ʾ����������������õĿ���ֵ
 *                  unsigned int    nMin;           // ��ʾ�����������С�����õĿ���ֵ
 *                  unsigned int    nInc;           // ��ʾ������õĿ�������������nInc�ı�����������Ч
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          �������ͽṹ������Ľӿڿɲ��մ˽ӿ�
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetWidth(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
*  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const unsigned int nValue);
*  @brief  ����ͼ�����
*  @param  void* handle                [IN]        ������
*          const unsigned int nValue   [IN]        ��Ҫ���õ�������ȵ�ֵ,ע��˿���ֵ������MV_CC_GetWidth�ӿڷ��ص�pstValue�е�nInc�ı����������óɹ�
*  @return �ɹ�,����MV_OK,����������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetWidth(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��߶�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����߶ȵ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�����߶���Ϣ���ص��ṹ���У�ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeight(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����ͼ��߶�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�������ȵ�ֵ,ע��˿���ֵ������MV_CC_GetWidth�ӿڷ��ص�pstValue�е�nInc�ı����������óɹ�
 *  @return �ɹ�,����MV_OK,��������߶Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeight(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��Xƫ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й����Xƫ�Ƶ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetX(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����ͼ��AOIƫ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����AOI��ֵ
 *  @return �ɹ�,����MV_OK,�������AOIƫ�ƽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡͼ��Yƫ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й����Yƫ�Ƶ���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAOIoffsetY(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetX(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����ͼ��AOIƫ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����AOI��ֵ
 *  @return �ɹ�,����MV_OK,�������AOIƫ�ƽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAOIoffsetY(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�ع�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����ع�ֵ���޽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeLower(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void* handle, IN const unsigned int nValue);
 *  @brief  �����ع�ֵ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��ع�ֵ����
 *  @return �ɹ�,����MV_OK,��������ع����޽������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeLower(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�ع�����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����ع�ֵ���޽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAutoExposureTimeUpper(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void* handle, IN const unsigned int nValue);
 *  @brief  �����ع�ֵ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��ع�ֵ����
 *  @return �ɹ�,����MV_OK,��������ع����޽������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAutoExposureTimeUpper(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ����ֵ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������Ƚṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBrightness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN const unsigned int nValue);
 *  @brief  ��������ֵ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�����ֵ
 *  @return �ɹ�,����MV_OK,����������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBrightness(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ֡��
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   ���ظ��������й����֡�ʵ���Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      // ��ʾ�����ǰ��֡��
 *                                      float           fMax;           // ��ʾ����������õ����֡��
 *                                      float           fMin;           // ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameRate(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN const float fValue);
 *  @brief  ����֡��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameRate(IN void* handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ����
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   ���ظ��������й�����������Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      // ��ʾ�����ǰ��֡��
 *                                      float           fMax;           // ��ʾ����������õ����֡��
 *                                      float           fMin;           // ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGain(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const float fValue);
 *  @brief  ����֡��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGain(IN void* handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ�ع�ʱ��
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   ���ظ��������й�����ع�ʱ�����Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *                                      float           fCurValue;      // ��ʾ�����ǰ��֡��
 *                                      float           fMax;           // ��ʾ����������õ����֡��
 *                                      float           fMin;           // ��ʾ����������õ���С֡��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          ���������ͽṹ������Ľӿڿɲ��մ˽ӿ�
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureTime(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle, IN const float fValue);
 *  @brief  �����ع�ʱ��
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����֡��
 *  @return �ɹ�,����MV_OK,�������֡�ʽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureTime(IN void* handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ���ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��й����ظ�ʽ����Ϣ�ṹ��ָ��
 *          ���ص�pstValue�ṹ�������
 *          unsigned int    nCurValue;                              //  �����ǰ�����ظ�ʽ����ö������,����˵PixelType_Gvsp_Mono8, �����õ���������ֵ,������ֵ����PixelType.h��MvGvspPixelTypeö������
 *          unsigned int    nSupportedNum;                          //  ���֧�ֵ����ظ�ʽ�ĸ���
 *          unsigned int    nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]; //  �������֧�ֵ����ظ�ʽ��Ӧ������ֵ�б�������Ҫ�������ظ�ʽʱ��������������������е�һ�֣�������Ч
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            ����ö�����Ͳ����ӿڿɲ��մ˽ӿڣ��й���Ӧ������ö�����Ͷ�Ӧ������ֵ�����PixelType.h �� CameraParams.h����Ӧ�Ķ���
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetPixelFormat(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle, IN const unsigned int nValue);
 *  @brief  �������ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ����ظ�ʽ��Ӧ������ֵ�����ô˽ӿ�ʱ����ֱ����дö��ֵ����MV_CC_SetPixelFormat(m_handle, PixelType_Gvsp_RGB8_Packed);
 *  @return �ɹ�,����MV_OK,����������ظ�ʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 *  
 *          Ҫ���õ�ö�����ͱ�����Get�ӿڷ��ص�nSupportValue[MV_MAX_XML_SYMBOLIC_NUM]�е�һ�֣������ʧ��
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetPixelFormat(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�ɼ�ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��йزɼ�ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_ACQUISITION_MODE ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  �������ظ�ʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĲɼ�ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������ɼ�ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionMode(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��й�����ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_GAIN_MODE ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGainMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  ��������ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ�����ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGainMode(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�Զ��ع�ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��й��Զ��ع�ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_EXPOSURE_AUTO_MODE ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetExposureAutoMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  �����Զ��ع�ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ��Զ��ع�ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������Զ��ع�ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetExposureAutoMode(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����ģʽ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��йش���ģʽ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_TRIGGER_MODE ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerMode(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ô���ģʽ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĴ���ģʽ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������ģʽ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerMode(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡ������ʱ
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   ���ظ��������й����������ʱ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerDelay(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle, IN const float fValue);
 *  @brief  ���ô�����ʱ
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����������ʱ
 *  @return �ɹ�,����MV_OK,�������������ʱ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerDelay(IN void* handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ����Դ
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��йش���Դ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_TRIGGER_SOURCE ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetTriggerSource(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ô���Դ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õĴ���Դ��Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�����������Դ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetTriggerSource(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void* handle);
 *  @brief  ������һ�Σ��ӿڽ�����ѡ��Ĵ���ԴΪ��������ʱ��Ч��
 *  @param  void* handle                [IN]        ������
 *  @return �ɹ�,����MV_OK, ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_TriggerSoftwareExecute(IN void* handle);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡGamma����
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��й�Gamma���͵���Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_GAMMA_SELECTOR ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGammaSelector(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����Gamma����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ�Gamma���Ͷ�Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,�������Gamma���ͽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaSelector(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);
 *  @brief  ��ȡGammaֵ
 *  @param  void* handle                [IN]        ������
 *          MVCC_FLOATVALUE* pstValue   [IN][OUT]   ���ظ��������й����Gammaֵ����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetFrameRate
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetGamma(IN void* handle, IN OUT MVCC_FLOATVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN const float fValue);
 *  @brief  ����Gammaֵ
 *  @param  void* handle                [IN]        ������
 *          const float fValue          [IN]        ��Ҫ���õ����Gammaֵ
 *  @return �ɹ�,����MV_OK,�������Gammaֵ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGamma(IN void* handle, IN const float fValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ���
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������Ƚṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSharpness(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN const unsigned int nValue);
 *  @brief  �������
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ����
 *  @return �ɹ�,����MV_OK,���������Ƚ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSharpness(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�Ҷ�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����ҶȽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ûҶ�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õĻҶ�
 *  @return �ɹ�,����MV_OK,��������ҶȽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHue(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ���Ͷ�
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�������ͶȽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetSaturation(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ñ��Ͷ�
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õı��Ͷ�
 *  @return �ɹ�,����MV_OK,����������ͶȽ������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetSaturation(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);
 *  @brief  ��ȡ�Զ���ƽ��
 *  @param  void* handle                [IN]        ������
 *          MVCC_ENUMVALUE* pstValue    [IN][OUT]   ���ظ������ߵ��й��Զ���ƽ�����Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,�������Ӧ������Ϣ�Ľṹ��, ʧ��, ���ش�����
 *  
            �ɲ��սӿ�MV_CC_GetPixelFormat���ο� CameraParam.h �е� MV_CAM_BALANCEWHITE_AUTO ����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceWhiteAuto(IN void* handle, IN OUT MVCC_ENUMVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void* handle, IN const unsigned int nValue);
 *  @brief  �����Զ���ƽ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        Ҫ���õ��Զ���ƽ���Ӧ������ֵ
 *  @return �ɹ�,����MV_OK,��������Զ���ƽ�⽫�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceWhiteAuto(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ�� ��ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioRed(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� �콫�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioRed(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ�� �̽ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioGreen(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� �̽������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioGreen(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������ƽ�� ���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBalanceRatioBlue(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void* handle, IN const unsigned int nValue);
 *  @brief  ���ð�ƽ�� ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õİ�ƽ�� ��
 *  @return �ɹ�,����MV_OK,���������ƽ�� ���������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBalanceRatioBlue(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡˮӡ��Ϣ�ڰ�������Ϣ����
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й����ˮӡ��Ϣ�ڰ�������Ϣ���ͽṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFrameSpecInfoAbility(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����ˮӡ��Ϣ�ڰ�������Ϣ����
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�ˮӡ��Ϣ�ڰ�������Ϣ����
 *  @return �ɹ�,����MV_OK,�������ˮӡ��Ϣ�ڰ�������Ϣ���ͻ����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFrameSpecInfoAbility(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle, IN OUT MVCC_STRINGVALUE* pstValue);
 *  @brief  ��ȡ�豸�Զ�������
 *  @param  void* handle                [IN]        ������
 *          MVCC_STRINGVALUE* pstValue  [IN OUT]    ���ظ��������й�������ֽṹ��ָ��
 *  @return �ɹ�,����MV_OK,���һ�ȡ��������Զ������֣�ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceUserID(IN void* handle, IN OUT MVCC_STRINGVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle, IN const char* chValue);
 *  @brief  �����豸�Զ�������
 *  @param  void* handle                [IN]        ������
 *          IN const char* chValue      [IN]        �豸����
 *  @return �ɹ�,����MV_OK,���������豸�Զ������֣�ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetDeviceUserID(IN void* handle, IN const char* chValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡһ�δ�����֡��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й����һ�δ�����֡���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBurstFrameCount(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����һ�δ�����֡��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�һ�δ�����֡��
 *  @return �ɹ�,����MV_OK,�������һ�δ�����֡�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBurstFrameCount(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��Ƶ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������Ƶ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAcquisitionLineRate(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void* handle, IN const unsigned int nValue);
 *  @brief  ������Ƶ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ���Ƶ
 *  @return �ɹ�,����MV_OK,���������Ƶ�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetAcquisitionLineRate(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ������Ϣ
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й����������Ϣ�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetHeartBeatTimeout(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void* handle, IN const unsigned int nValue);
 *  @brief  ����������Ϣ
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ�������Ϣ
 *  @return �ɹ�,����MV_OK,�������������Ϣ�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetHeartBeatTimeout(IN void* handle, IN const unsigned int nValue);




/************************************************************************/
/* �豸���� �� �Ĵ�����д ���쳣���¼��ص�                            */
/************************************************************************/
// �豸��������
/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle, 
                                                            const void *pFilePathName);
 *  @brief  �豸��������
 *  @param  void* handle                  [IN]        ������
 *  @param  void *pFilePathName           [IN]        �ļ���
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle, const void *pFilePathName);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void* handle,
                                                                 unsigned int* pnProcess);
 *  @brief  ��ȡ��������
 *  @param  void* handle                  [IN]        ������
 *  @param  unsigned int* pnProcess       [OUT]       ���Ƚ��յ�ַ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void* handle, unsigned int* pnProcess);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void* handle);
 *  @brief  ��ȡ��ѵ�packet size����ӦGigEVision�豸�� SCPS����ӦU3V�豸��ÿ�δ�������ȡ�İ���С
 *  @param  void* handle                  [IN]        ������
 *  @return ���packetsize
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void* handle);

/************************************************************************
 *  @fn     MV_CC_ReadMemory
 *  @brief  ���ڴ�
 *  @param  handle���豸���
 *  @param  pBuffer����Ϊ����ֵʹ�ã�����������ڴ�ֵ���ڴ�ֵ�ǰ��մ��ģʽ�洢�ģ�
 *  @param  nAddress������ȡ���ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                  ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
 *  @param  nLength������ȡ���ڴ泤��
 *  @return �����ش�����
*************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReadMemory(IN void* handle , void *pBuffer, int64_t nAddress, int64_t nLength);

/************************************************************************
 *  @fn     MV_CC_WriteMemory
 *  @brief  д�ڴ�
 *  @param  handle���豸���
 *  @param  pBuffer����д����ڴ�ֵ��ע���ڴ�ֵҪ���մ��ģʽ�洢��
 *  @param  nAddress����д����ڴ��ַ���õ�ַ���Դ��豸��Camera.xml�ļ��л�ȡ������xxx_RegAddr��xml�ڵ�ֵ
                  ���豸��Camera.xml�ļ������豸��֮���Զ�������Ӧ�ó���ĵ�ǰĿ¼�У�
 *  @param  nLength����д����ڴ泤��
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_WriteMemory(IN void* handle , const void *pBuffer, int64_t nAddress, int64_t nLength);

// ע���쳣��Ϣ�ص����ڴ��豸֮�����
/************************************************************************
 *  @fn     MV_CC_RegisterExceptionCallBack
 *  @brief  ע���쳣��Ϣ�ص����ڴ��豸֮�����
 *  @param  handle���豸���
 *  @param  cbException       [IN]      �쳣�ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterExceptionCallBack(IN void* handle, 
                                                             void(__stdcall* cbException)(unsigned int nMsgType, void* pUser),
                                                             void* pUser);

/************************************************************************
 *  @fn     MV_CC_RegisterEventCallBack
 *  @brief  ע���¼��ص����ڴ��豸֮�����,ֻ֧��GIGE
 *  @param  handle���豸���
 *  @param  cbEvent           [IN]      �쳣�ص�����ָ��
 *  @param  pUser             [IN]      �û��Զ������
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBack(void* handle, 
                                                         void(__stdcall* cbEvent)(unsigned int nExternalEventId, void* pUser),
                                                         void* pUser);




/************************************************************************/
/* GigEVision �豸���еĽӿ�                                     */
/************************************************************************/
/************************************************************************
 *  @fn     MV_GIGE_ForceIpEx
 *  @brief  ǿ��IP
 *  @param  handle���豸���
 *  @param  nIP               [IN]      ���õ�IP
 *  @param  nSubNetMask       [IN]      ��������
 *  @param  nDefaultGateWay   [IN]      Ĭ������
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIpEx(IN void* handle, unsigned int nIP, unsigned int nSubNetMask, unsigned int nDefaultGateWay);

/************************************************************************
 *  @fn     MV_GIGE_SetIpConfig
 *  @brief  ����IP��ʽ
 *  @param  handle���豸���
 *  @param  nType               [IN]      IP���ͣ���MV_IP_CFG_x
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetIpConfig(IN void* handle, unsigned int nType);

/************************************************************************
 *  @fn     MV_GIGE_SetNetTransMode
 *  @brief  ���ý�ʹ��ĳ��ģʽ,type: MV_NET_TRANS_x��������ʱ��Ĭ������ʹ��driver
 *  @param  handle���豸���
 *  @param  nType               [IN]      ���紫��ģʽ����MV_NET_TRANS_x
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetNetTransMode(IN void* handle, unsigned int nType);

/************************************************************************
 *  @fn     MV_GIGE_GetNetTransInfo
 *  @brief  ��ȡ���紫����Ϣ
 *  @param  handle���豸���
 *  @param  pstInfo             [OUT]      ��Ϣ�ṹ��
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetNetTransInfo(IN void* handle, MV_NETTRANS_INFO* pstInfo);

/************************************************************************
 *  @fn     MV_GIGE_SetGvcpTimeout
 *  @brief  ����GVCP���ʱʱ��
 *  @param  handle                 [IN]           �����ַ
 *  @param  nMillisec              [IN]           ��ʱʱ�䣬�Ժ���λ��λ����Χ��0-10000
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvcpTimeout(void* handle, unsigned int nMillisec);

/************************************************************************
 *  @fn     MV_GIGE_SetResend
 *  @brief  �����Ƿ���ط���֧�֣����ط�������
 *  @param  handle                 [IN]           �����ַ
 *  @param  bEnable                [IN]           �Ƿ�֧���ط���
 *  @param  nMaxResendPercent      [IN]           ����ط���
 *  @param  nResendTimeout         [IN]           �ط���ʱʱ��
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(void* handle, unsigned int bEnable, unsigned int nMaxResendPercent = 10, unsigned int nResendTimeout = 50);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ�������С
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й�����������С�ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPSPacketSize(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void* handle, IN const unsigned int nValue);
 *  @brief  �����������С
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ��������С
 *  @return �ɹ�,����MV_OK,��������������С�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPSPacketSize(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);
 *  @brief  ��ȡ��������ͼ��
 *  @param  void* handle                [IN]        ������
 *  @param  MVCC_INTVALUE* pstValue     [IN][OUT]   ���ظ��������й������������ͼ���ṹ��ָ��
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 *  
 *          �ɲ��սӿ�MV_CC_GetWidth
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCPD(IN void* handle, IN OUT MVCC_INTVALUE* pstValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN const unsigned int nValue);
 *  @brief  ������������ͼ��
 *  @param  void* handle                [IN]        ������
 *          const unsigned int nValue   [IN]        ��Ҫ���õ���������ͼ��
 *  @return �ɹ�,����MV_OK,���������������ͼ�������Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCPD(IN void* handle, IN const unsigned int nValue);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle, unsigned int* pnIP);
 *  @brief  ��ȡ���ն�IP��ַ��0xa9fe0102 ��ʾ 169.254.1.2
 *  @param  void* handle                [IN]        ������
 *  @param  unsigned int* pnIP          [IN][OUT]   ���ظ������߽��ն�IP��ַ
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCDA(IN void* handle, unsigned int* pnIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle, unsigned int nIP);
 *  @brief  ���ý��ն�IP��ַ
 *  @param  void* handle                [IN]        ������
 *          unsigned int nIP            [IN]        ��Ҫ���õĽ��ն�IP��ַ
 *  @return �ɹ�,����MV_OK,����������ն�IP��ַ�����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCDA(IN void* handle, unsigned int nIP);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle, unsigned int* pnPort);
 *  @brief  ��ȡ���Ͷ˵Ķ˿ں�
 *  @param  void* handle                [IN]        ������
 *  @param  unsigned int* pnPort        [IN][OUT]   ���ظ������߷��Ͷ˵Ķ˿ں�
 *  @return �ɹ�,����MV_OK,ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGevSCSP(IN void* handle, unsigned int* pnPort);

/************************************************************************
 *  @fn     MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle, unsigned int nPort);
 *  @brief  ���÷��Ͷ˵Ķ˿ں�
 *  @param  void* handle                [IN]        ������
 *          unsigned int nPort          [IN]        ��Ҫ���õķ��Ͷ˵Ķ˿ں�
 *  @return �ɹ�,����MV_OK,����������Ͷ˵Ķ˿ںŻ����Ϊ��Ӧֵ��ʧ��,���ش�����
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGevSCSP(IN void* handle, unsigned int nPort);




/************************************************************************/
/* XML������������                                                         */
/************************************************************************/
/***********************************************************************
 *  @fn         MV_XML_GetGenICamXML
 *  @brief      ��ȡ���������XML
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pnDataLen              [OUT]         ʵ�����ݴ�С
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetGenICamXML(IN void* handle, IN OUT unsigned char* pData, IN unsigned int nDataSize, OUT unsigned int* pnDataLen);

/***********************************************************************
 *  @fn         MV_XML_GetRootNode
 *  @brief      ��ȡ���ڵ�
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [OUT]         ���ڵ���Ϣ�ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetRootNode(IN void* handle, IN OUT MV_XML_NODE_FEATURE* pstNode);

/***********************************************************************
 *  @fn         MV_XML_GetChildren
 *  @brief      ��xml�л�ȡָ���ڵ�������ӽڵ㣬���ڵ�ΪRoot
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [IN]          ���ڵ���Ϣ�ṹ��
 *  @param       pstNodesList           [OUT]         �ڵ��б��ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetChildren(IN void* handle, IN MV_XML_NODE_FEATURE* pstNode, IN OUT MV_XML_NODES_LIST* pstNodesList);

/***********************************************************************
 *  @fn         MV_XML_GetNodeFeature
 *  @brief      ��õ�ǰ�ڵ������
 *  @param       handle                 [IN]          ���
 *  @param       pstNode                [IN]          ���ڵ���Ϣ�ṹ��
 *  @param       pstFeature             [OUT]         ��ǰ�ڵ����Խṹ�壬
                           pstFeature ����ṹ�����ݲο� MV_XML_FEATURE_x
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeFeature(IN void* handle, IN MV_XML_NODE_FEATURE* pstNode, IN OUT void* pstFeature);

/***********************************************************************
 *  @fn         MV_XML_UpdateNodeFeature
 *  @brief      ���½ڵ�
 *  @param       handle                 [IN]          ���
 *  @param       enType                 [IN]          �ڵ�����
 *  @param       pstFeature             [OUT]         ��ǰ�ڵ����Խṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_UpdateNodeFeature(IN void* handle, IN enum MV_XML_InterfaceType enType, IN void* pstFeature);

// �нڵ���Ҫ����ʱ�Ļص�����
// ������MV_XML_UpdateNodeFeature�ӿڸ��½ڵ�����ʱ��ע��Ļص�����cbUpdate����pstNodesList�з�����֮������Ľڵ�
/***********************************************************************
 *  @fn         MV_XML_RegisterUpdateCallBack
 *  @brief      ע����»ص�
 *  @param       handle                 [IN]          ���
 *  @param       cbUpdate               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_RegisterUpdateCallBack(IN void* handle, 
                                                           IN void(__stdcall* cbUpdate)(enum MV_XML_InterfaceType enType, void* pstFeature, MV_XML_NODES_LIST* pstNodesList, void* pUser),
                                                           IN void* pUser);




/************************************************************************/
/* ���ӽӿ�                                   */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_SaveImageEx
 *  @brief  ����ͼƬ��֧��Bmp��Jpeg.����������50-99֮ǰ
 *  @param  pSaveParam             [IN][OUT]          ����ͼƬ�����ṹ��
                       pData;              // [IN]     �������ݻ���
                       nDataLen;           // [IN]     �������ݴ�С
                       enPixelType;        // [IN]     �������ݵ����ظ�ʽ
                       nWidth;             // [IN]     ͼ���
                       nHeight;            // [IN]     ͼ���
                       pImageBuffer;       // [OUT]    ���ͼƬ����
                       nImageLen;          // [OUT]    ���ͼƬ��С
                       nBufferSize;        // [IN]     �ṩ�������������С
                       enImageType;        // [IN]     ���ͼƬ��ʽ
                       nJpgQuality;        // [IN]     ��������, (50-99]
                       nReserved[4];
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx(IN OUT MV_SAVE_IMAGE_PARAM_EX* pSaveParam);

/************************************************************************
 *  @fn     MV_CC_ConvertPixelType
 *  @brief  ���ظ�ʽת��
 *  @param  pstCvtParam             [IN][OUT]          ����ͼƬ�����ṹ��
                     unsigned short         nWidth;             // [IN]     ͼ���
                     unsigned short         nHeight;            // [IN]     ͼ���
                     enum MvGvspPixelType   enSrcPixelType;     // [IN]     Դ���ظ�ʽ
                     unsigned char*         pSrcData;           // [IN]     �������ݻ���
                     unsigned int           nSrcDataLen;        // [IN]     �������ݴ�С
                     enum MvGvspPixelType   enDstPixelType;     // [IN]     Ŀ�����ظ�ʽ
                     unsigned char*         pDstBuffer;         // [OUT]    ������ݻ���
                     unsigned int           nDstLen;            // [OUT]    ������ݴ�С
                     unsigned int           nDstBufferSize;     // [IN]     �ṩ�������������С
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelType(IN void* handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM* pstCvtParam);

/************************************************************************
 *  @fn     MV_CC_SetBayerCvtQuality
 *  @brief  ��ֵ�㷨��������
 *  @param  BayerCvtQuality             [IN]          Bayer�Ĳ�ֵ����  0-����� 1-˫���� 2-Hamilton
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCvtQuality(IN void* handle, IN unsigned int BayerCvtQuality);

/************************************************************************
 *  @fn     MV_CC_GetTlProxy
 *  @brief  ��ȡGenICam����
 *  @param  handle                 [IN]           �����ַ
 *  @return GenICam������ָ�� ����������ֵ��NULL���쳣����NULL
 ************************************************************************/
MV_CAMCTRL_API void* __stdcall MV_CC_GetTlProxy(IN void* handle);




/************************************************************************/
/* ����ݲ�֧�ֵĽӿ�                                 */
/************************************************************************/
/************************************************************************
 *  @fn     MV_CC_FeatureSave
 *  @brief  �����������
 *  @param  handle                [IN]           �����ַ
 *  @param  pFileName             [IN]          �����ļ���
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureSave(IN void* handle, IN const char* pFileName);

/************************************************************************
 *  @fn     MV_CC_FeatureLoad
 *  @brief  �����������
 *  @param  handle                [IN]           �����ַ
 *  @param  pFileName             [IN]          �����ļ���
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoad(IN void* handle, IN const char* pFileName);




/************************************************************************/
/* ���õĽӿ�                                 */
/************************************************************************/
/***********************************************************************
 *  @fn         MV_CC_GetOneFrame
 *  @brief      ��ȡһ֡ͼ�񣬴˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�������
                �����ݣ���������Χ���ݣ������ݷ��ش�����
                ���ýӿ������ã�������� MV_CC_GetOneFrameTimeOut�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrame(IN void* handle, IN OUT unsigned char * pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO* pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_GetOneFrameEx
 *  @brief      ��ȡһ֡trunck���ݣ��˺���Ϊ��ѯʽ��ȡ��ÿ�ε��ò�ѯ�ڲ�
                �����������ݣ���������Χ���ݣ������ݷ��ش�����
                ���ýӿ������ã�������� MV_CC_GetOneFrameTimeOut�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       pData                  [OUT]         ͼ�����ݽ���ָ��
 *  @param       nDataSize              [IN]          ���ջ����С
 *  @param       pFrameInfo             [OUT]         ͼ����Ϣ�ṹ��
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameEx(IN void* handle, IN OUT unsigned char * pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pFrameInfo);

/***********************************************************************
 *  @fn         MV_CC_RegisterImageCallBack
 *  @brief      ע��ͼ�����ݻص����ýӿ������ã�������� MV_CC_RegisterImageCallBackEx�ӿڣ�
 *  @param       handle                 [IN]          ���
 *  @param       cbOutput               [IN]          �ص�����ָ��
 *  @param       pUser                  [IN]          �û��Զ������
 *  @return �ɹ�������MV_OK�����󣬷��ش�����
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBack(void* handle, 
                                                         void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO* pFrameInfo, void* pUser),
                                                         void* pUser);

/************************************************************************
 *  @fn     MV_CC_SaveImage
 *  @brief  ����ͼƬ���ýӿ������ã�������� MV_CC_SaveImageEx�ӿڣ�
 *  @param  pSaveParam             [IN][OUT]          ����ͼƬ�����ṹ��
                       pData;              // [IN]     �������ݻ���
                       nDataLen;           // [IN]     �������ݴ�С
                       enPixelType;        // [IN]     �������ݵ����ظ�ʽ
                       nWidth;             // [IN]     ͼ���
                       nHeight;            // [IN]     ͼ���
                       pImageBuffer;       // [OUT]    ���ͼƬ����
                       nImageLen;          // [OUT]    ���ͼƬ��С
                       nBufferSize;        // [IN]     �ṩ�������������С
                       enImageType;        // [IN]     ���ͼƬ��ʽ
 *  @return �ɹ�������MV_OK�����󣬷��ش����� 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImage(IN OUT MV_SAVE_IMAGE_PARAM* pSaveParam);

/************************************************************************
 *  @fn     MV_GIGE_ForceIp
 *  @brief  ǿ��IP���ýӿ������ã�������� MV_GIGE_ForceIpEx�ӿڣ�
 *  @param  handle���豸���
 *  @param  nIP               [IN]      ���õ�IP
 *  @return �����ش�����
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIp(IN void* handle, unsigned int nIP);



#ifdef __cplusplus
}
#endif 

#endif //_MV_CAMERA_CTRL_H_
