#include "spiffs_user.h"


static s32_t core_spiflash_spiffs_read(u32_t addr, u32_t size, u8_t *dst);
static s32_t core_spiflash_spiffs_write(u32_t addr, u32_t size, u8_t *src);
static s32_t core_spiflash_spiffs_erase(u32_t addr, u32_t size);

/*�ļ�ϵͳ�ṹ��*/
spiffs fs;

/*ҳ����*/
#define LOG_PAGE_SIZE		256

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[32*4];
static u8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];

/*
	�ļ�ϵͳ����
	
	#define SPIFFS_SINGLETON (0) ���������Ϊ0��Ҳ����֧�ֶ��spiffs
*/
spiffs_config cfg=
	{	
		/* �����SPIFFS�Ŀռ� Ҫ���߼�������������*/
		.phys_size = 1024*4*4*256,
		/* ��ʼ��ַ */
		.phys_addr = 0,
		/*
			����������Ҳ����һ�β����Ĵ�С��Ҫ��hal_erase_f����������һ��
		*/
		.phys_erase_block = 1024*4, 
		/* �߼�����������������������������
			�����: log_block_size/log_page_size = ��32-512��
		*/
		.log_block_size = 1024*4*4,
		/*�߼�ҳ��ͨ����256*/
		.log_page_size = LOG_PAGE_SIZE, 
		
		.hal_read_f = core_spiflash_spiffs_read,
		.hal_write_f = core_spiflash_spiffs_write,
		.hal_erase_f = core_spiflash_spiffs_erase,
	
	};
/*
	��SPI FLASH�������ж��η�װ
	*/
/**
 *@brief:     
 *@details:  ��FLASH���� 
 *@param[in]    
 *@param[out]  
 *@retval:     
 */	
static s32_t core_spiflash_spiffs_read(u32_t addr, u32_t size, u8_t *dst)
{	
	return W25Qx_Read(dst, addr, size);
}
/**
 *@brief:     
 *@details: дFLASH����  
 *@param[in]    
 *@param[out]  
 *@retval:     
 */
static s32_t core_spiflash_spiffs_write(u32_t addr, u32_t size, u8_t *src)
{
	return W25Qx_Write(src, addr, size);
}
/**
 *@brief:     
 *@details:  ����
 			 SPIFFS�ᰴ��spiffs_config��.phys_erase_blockֵ���в���
 			 ���Բ��ù���size
 *@param[in]    
 *@param[out]  
 *@retval:     
 */

static s32_t core_spiflash_spiffs_erase(u32_t addr, u32_t size)
{
	printf("%d\r\n",size);
	return W25Qx_Erase_Block(addr);
}

/**
 *@brief:     
 *@details: ��ʽ���ļ�ϵͳ
 *@param[in]    
 *@param[out]  
 *@retval:     
 */
int32_t sys_spiffs_format(void)
{

	//wjq_log(LOG_INFO, ">---format spiffs coreflash\r\n");

	/* ��ʽ��֮ǰҪ��unmount */
	SPIFFS_unmount(&fs);
			
	SPIFFS_format(&fs);

	int res = SPIFFS_mount(&fs,
	  &cfg,
	  spiffs_work_buf,
	  spiffs_fds,
	  sizeof(spiffs_fds),
	  spiffs_cache_buf,
	  sizeof(spiffs_cache_buf),
	  0);
	printf("mount res: %i\r\n", res);
	
	printf(">---format spiffs coreflash finish\r\n");
	return res;
}

/**
 *@brief:     
 *@details: ����spiffs�ļ�ϵͳ
 *@param[in]    
 *@param[out]  
 *@retval:     
 */
int32_t sys_spiffs_mount_coreflash(void) 
{ 
    int res = SPIFFS_mount(&fs,
      						&cfg,
      						spiffs_work_buf,
      						spiffs_fds,
      						sizeof(spiffs_fds),
      						spiffs_cache_buf,
     		 				sizeof(spiffs_cache_buf),
      						0);
   printf("mount res: %i\r\n", res);

	if(SPIFFS_ERR_NOT_A_FS == res )
	{
		res = sys_spiffs_format();	
	}
	return res;
 }
/**
 *@brief:     
 *@details: �г��ļ�ϵͳ�ļ�
 *@param[in]    
 *@param[out]  
 *@retval:     
 */
void sys_spiffs_ls(void)
{
	spiffs_DIR d;
	struct spiffs_dirent e;
	struct spiffs_dirent *pe = &e;

	printf(">---ls spiffs file\r\n");
	SPIFFS_opendir(&fs, "/", &d);
	while ((pe = SPIFFS_readdir(&d, pe)) != 0) 
	{
		printf("%s [%04x] size:%i\r\n", pe->name, pe->obj_id, pe->size);
	}
	SPIFFS_closedir(&d);	
	printf(">---ls spiffs file end ---\r\n");
}

void test_spiffs(void)
{
	char buf[12];
	spiffs_file fd;
  fd = SPIFFS_open(&fs, "my_file", SPIFFS_RDWR, 0);
  if (fd < 0) {
	  // create a file, delete previous if it already exists, and open it for reading and writing
	  fd = SPIFFS_open(&fs, "my_file", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_RDWR, 0);
	  if (fd < 0) {
		 printf("errno %i\n", SPIFFS_errno(&fs));
		 return;
	  }
	  // write to it
	  if (SPIFFS_write(&fs, fd, (u8_t *)"Hello world", 12) < 0) {
		 printf("errno %i\n", SPIFFS_errno(&fs));
		 return;
	  }	  
    //printf("errno %i\n", SPIFFS_errno(&fs));
    //return;
  }
  // read it
  if (SPIFFS_read(&fs, fd, (u8_t *)buf, 12) < 0) {

    printf("errno %i\n", SPIFFS_errno(&fs));
    //return;
  }

  // close it
  if (SPIFFS_close(&fs, fd) < 0) {
    printf("errno %i\n", SPIFFS_errno(&fs));
    return;
  }

  // check it
  printf("--> %s <--\n", buf);
}
