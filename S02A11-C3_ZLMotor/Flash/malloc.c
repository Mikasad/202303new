#include "public_h.h"	   


//�ڴ��(32�ֽڶ���)
__align(32) u8 mem1base[MEM1_MAX_SIZE];									//�ڲ�SRAM�ڴ��
//�ڴ������
u32 mem1mapbase[MEM1_ALLOC_TABLE_SIZE];									//�ڲ�SRAM�ڴ��MAP
//�ڴ��������	   
const u32 memtblsize[SRAMBANK]={MEM1_ALLOC_TABLE_SIZE};	//�ڴ����С
const u32 memblksize[SRAMBANK]={MEM1_BLOCK_SIZE};				//�ڴ�ֿ��С
const u32 memsize[SRAMBANK]={MEM1_MAX_SIZE};						//�ڴ��ܴ�С

//�ڴ����������
struct _m_mallco_dev mallco_dev=
{
	my_mem_init,						//�ڴ��ʼ��
	my_mem_perused,					//�ڴ�ʹ����
	mem1base,						    //�ڴ��
	mem1mapbase,				    //�ڴ����״̬��
	0,  		 						    //�ڴ����δ����
};

/*----------------------------
*Fuction�������ڴ�
*Explain��*des:Ŀ�ĵ�ַ
					*src:Դ��ַ
					n:��Ҫ���Ƶ��ڴ泤��(�ֽ�Ϊ��λ)
----------------------------*/
void mymemcpy(void *des,void *src,u32 n)  
{  
    u8 *xdes=des;
	  u8 *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}
/*----------------------------
*Fuction�������ڴ�
*Explain��*s:�ڴ��׵�ַ
					c :Ҫ���õ�ֵ
					count:��Ҫ���õ��ڴ��С(�ֽ�Ϊ��λ)
----------------------------*/
void mymemset(void *s,u8 c,u32 count)  
{  
    u8 *xs = s;  
    while(count--)*xs++=c;  
}	
/*----------------------------
*Fuction���ڴ������ʼ�� 
*Explain��memx:�����ڴ��
----------------------------*/
void my_mem_init(u8 memx)  
{  
    mymemset(mallco_dev.memmap[memx],0,memtblsize[memx]*4);	//�ڴ�״̬����������  
 	  mallco_dev.memrdy[memx]=1;															//�ڴ������ʼ��OK  
}  
/*----------------------------
*Fuction����ȡ�ڴ�ʹ����
*Explain��memx:�����ڴ��
*Output : ʹ����(������10��,0~1000,����0.0%~100.0%)
----------------------------*/
u16 my_mem_perused(u8 memx)  
{  
    u32 used=0;  
    u32 i;  
    for(i=0;i<memtblsize[memx];i++)  
    {  
        if(mallco_dev.memmap[memx][i])used++; 
    } 
    return (used*1000)/(memtblsize[memx]);  
}  
/*----------------------------
*Fuction���ڴ����(�ڲ�����)
*Explain��memx:�����ڴ��
					size:Ҫ������ڴ��С(�ֽ�)
*Output : 0XFFFFFFFF,��������;����,�ڴ�ƫ�Ƶ�ַ 
----------------------------*/
u32 my_mem_malloc(u8 memx,u32 size)  
{  
    signed long offset=0;  
    u32 nmemb;																				//��Ҫ���ڴ����  
		u32 cmemb=0;																			//�������ڴ����
    u32 i;  
    if(!mallco_dev.memrdy[memx])mallco_dev.init(memx);//δ��ʼ��,��ִ�г�ʼ��
    if(size==0)return 0XFFFFFFFF;											//����Ҫ����
    nmemb=size/memblksize[memx];  										//��ȡ��Ҫ����������ڴ����
    if(size%memblksize[memx])nmemb++;  
    for(offset=memtblsize[memx]-1;offset>=0;offset--) //���������ڴ������  
    {     
		if(!mallco_dev.memmap[memx][offset])cmemb++;			//�������ڴ��������
		else cmemb=0;																			//�����ڴ������
		if(cmemb==nmemb)																	//�ҵ�������nmemb�����ڴ��
		{
            for(i=0;i<nmemb;i++)  										//��ע�ڴ��ǿ� 
            {  
                mallco_dev.memmap[memx][offset+i]=nmemb;  
            }  
            return (offset*memblksize[memx]);					//����ƫ�Ƶ�ַ  
		}
    }  
    return 0XFFFFFFFF;																//δ�ҵ����Ϸ����������ڴ��  
} 
/*----------------------------
*Fuction���ͷ��ڴ�(�ڲ�����) 
*Explain��memx:�����ڴ��
					offset:�ڴ��ַƫ��
*Output : 0,�ͷųɹ�;1,�ͷ�ʧ��;  
----------------------------*/
u8 my_mem_free(u8 memx,u32 offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[memx])							//δ��ʼ��,��ִ�г�ʼ��
		{
				mallco_dev.init(memx);    
        return 1;															//δ��ʼ��  
    }  
    if(offset<memsize[memx])									//ƫ�����ڴ����. 
    {  
        int index=offset/memblksize[memx];		//ƫ�������ڴ�����  
        int nmemb=mallco_dev.memmap[memx][index];	//�ڴ������
        for(i=0;i<nmemb;i++)  							  //�ڴ������
        {  
            mallco_dev.memmap[memx][index+i]=0;  
        }  
        return 0;  
    }else return 2;														//ƫ�Ƴ�����.  
}  
/*----------------------------
*Fuction���ͷ��ڴ�(�ⲿ����) 
*Explain��memx:�����ڴ��
					ptr:�ڴ��׵�ַ  
----------------------------*/
void myfree(u8 memx,void *ptr)  
{  
	u32 offset;  
  u8 	free_status = 0;
	if(ptr==NULL)return;												//��ַΪ0.  
 	offset=(u32)ptr-(u32)mallco_dev.membase[memx];     
  free_status=my_mem_free(memx,offset);									  //�ͷ��ڴ�      
	printf("free_status=%d\r\n",free_status);   
}  
/*----------------------------
*Fuction�������ڴ�(�ⲿ����)
*Explain��memx:�����ڴ��
					size:�ڴ��С(�ֽ�)
*Output : ���䵽���ڴ��׵�ַ.
----------------------------*/
void *mymalloc(u8 memx,u32 size)  
{  
    u32 offset;   
	  offset=my_mem_malloc(memx,size);  	   	 	   
    if(offset==0XFFFFFFFF)return NULL;  
    else return (void*)((u32)mallco_dev.membase[memx]+offset);  
} 
/*----------------------------
*Fuction�����·����ڴ�(�ⲿ����)
*Explain��*ptr:���ڴ��׵�ַ
					size:Ҫ������ڴ��С(�ֽ�)
*Output : �·��䵽���ڴ��׵�ַ.
----------------------------*/
void *myrealloc(u8 memx,void *ptr,u32 size)  
{  
    u32 offset;    
    offset=my_mem_malloc(memx,size);   	
    if(offset==0XFFFFFFFF)return NULL;     
    else  
    {  									   
	    mymemcpy((void*)((u32)mallco_dev.membase[memx]+offset),ptr,size);	//�������ڴ����ݵ����ڴ�   
        myfree(memx,ptr);  											  											//�ͷž��ڴ�
        return (void*)((u32)mallco_dev.membase[memx]+offset);  					//�������ڴ��׵�ַ
    }  
}

/*----------------------------
*Fuction����̬�����ڴ�
*Explain��size:�����ڴ��С
----------------------------*/
void *ff_memalloc (unsigned  int size)			
{
	return (void*)mymalloc(SRAMIN,size);
}
/*----------------------------
*Fuction���ͷ��ڴ�
*Explain�����ͷŵ�ָ���ַ
----------------------------*/
void ff_memfree (void* mf)		 
{
	myfree(SRAMIN,mf);
}









