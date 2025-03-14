#ifndef __MEM_MANAGER__H__
#define __MEM_MANAGER__H__

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	int fd;
	unsigned long 	phys_addr;
	unsigned long	size;
	unsigned long 	virt_uaddr;
	int flag;
}udma_buf;

struct sm_mem_dev{
	udma_buf *mem;
};

struct sm_mem_dev *sm_mem_dev_create(int buf_num, int size);
void sm_mem_destroy(struct sm_mem_dev *mem_dev);
void sm_mem_buf_sync(struct sm_mem_dev *mem_dev, int no);
void sm_mem_free(struct sm_mem_dev *mem_dev, int buf_cnt);
int sm_mem_alloc(struct sm_mem_dev *mem_dev,int buf_cnt);
unsigned long sm_mem_get_viraddr(struct sm_mem_dev *mem_dev, int no);
unsigned long sm_mem_get_phyaddr(struct sm_mem_dev *mem_dev, int no);

#ifdef __cplusplus
}
#endif

#endif

