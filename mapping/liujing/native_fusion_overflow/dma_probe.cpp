#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdint>
#include <iostream>
int main(int argc,char**argv){
 if(argc!=2)return 2;int fd=std::atoi(argv[1]);void*base=mmap(nullptr,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);if(base==MAP_FAILED)return 3;
 dma_buf_sync access{DMA_BUF_SYNC_START|DMA_BUF_SYNC_RW};if(ioctl(fd,DMA_BUF_IOCTL_SYNC,&access))return 4;
 auto*words=static_cast<uint32_t*>(base);if(words[0]!=0x12345678)return 5;words[1]=0x89abcdef;
 access.flags=DMA_BUF_SYNC_END|DMA_BUF_SYNC_RW;if(ioctl(fd,DMA_BUF_IOCTL_SYNC,&access))return 6;
 munmap(base,4096);std::cout<<"PASS inherited native DMA-BUF mapping and CPU-access synchronization\n";return 0;
}
