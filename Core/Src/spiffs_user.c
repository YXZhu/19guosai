#include "spiffs_user.h"


static s32_t core_spiflash_spiffs_read(u32_t addr, u32_t size, u8_t *dst);
static s32_t core_spiflash_spiffs_write(u32_t addr, u32_t size, u8_t *src);
static s32_t core_spiflash_spiffs_erase(u32_t addr, u32_t size);

/*文件系统结构体*/
spiffs fs;

/*页定义*/
#define LOG_PAGE_SIZE		256

static u8_t spiffs_work_buf[LOG_PAGE_SIZE*2];
static u8_t spiffs_fds[32*4];
static u8_t spiffs_cache_buf[(LOG_PAGE_SIZE+32)*4];

/*
	文件系统配置
	
	#define SPIFFS_SINGLETON (0) 这个宏配置为0，也就是支持多个spiffs
*/
spiffs_config cfg=
	{	
		/* 分配给SPIFFS的空间 要是逻辑扇区的整数倍*/
		.phys_size = 1024*4*4*256,
		/* 起始地址 */
		.phys_addr = 0,
		/*
			物理扇区，也就是一次擦除的大小，要跟hal_erase_f函数擦除的一致
		*/
		.phys_erase_block = 1024*4, 
		/* 逻辑扇区，必须是物理扇区的整数倍
			最好是: log_block_size/log_page_size = （32-512）
		*/
		.log_block_size = 1024*4*4,
		/*逻辑页，通常是256*/
		.log_page_size = LOG_PAGE_SIZE, 
		
		.hal_read_f = core_spiflash_spiffs_read,
		.hal_write_f = core_spiflash_spiffs_write,
		.hal_erase_f = core_spiflash_spiffs_erase,
	
	};
/*
	对SPI FLASH函数进行二次封装
	*/
/**
 *@brief:     
 *@details:  读FLASH数据 
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
 *@details: 写FLASH数据  
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
 *@details:  擦除
 			 SPIFFS会按照spiffs_config中.phys_erase_block值进行操作
 			 所以不用关心size
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
 *@details: 格式化文件系统
 *@param[in]    
 *@param[out]  
 *@retval:     
 */
int32_t sys_spiffs_format(void)
{

	//wjq_log(LOG_INFO, ">---format spiffs coreflash\r\n");

	/* 格式化之前要先unmount */
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
 *@details: 挂载spiffs文件系统
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
 *@details: 列出文件系统文件
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
