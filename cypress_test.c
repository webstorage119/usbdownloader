#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

int main(int argc,char* argv[]){
    FILE* stream;
    char* dImageBuf;
    unsigned long size = 0;
    int ret = 0;
    struct stat statbuf;
    /* open Super speed USB FX3 downloader device */
    int fd = open("/dev/cypress_downloader0",O_RDWR);
    if(fd < 0){
        printf("[%s]Unable to open device\n",__func__);
        return -1;
    }
    /* file open the file *.img */
    if((stream = fopen("./cyfxuvc.img","r")) == NULL){
        printf("[%s]Can not open input file\n",__func__);
        close(fd);
        return -1;
    }
    dImageBuf = (char *)malloc(512 * 1024);
    /* calculate file size */
    if(stat("./cyfxuvc.img",&statbuf) < 0){
        printf("[%s]Can not stat file size\n",__func__);
    }
    size = statbuf.st_size;
    printf("[%s]stat file size is %ld\n",__func__,statbuf.st_size);
    fread(dImageBuf,sizeof(char),size,stream);
    /* write file *.img to device */
    if((ret = write(fd,dImageBuf,size)) != size){
        printf("[%s]Error in writing to the file\n",__func__);
        goto out;
    }
out:
    free(dImageBuf);
    close(fd);
    fclose(stream);
    return 0;
} 
