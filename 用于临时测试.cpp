#include<stdio.h>
#include<math.h>
int main()
{
	int x,y,z;
    float p,area;
	scanf("%d,%d,%d",&x,&y,&z);
	   p=(float)(x+y+z)/2;
	   area=sqrt(p*(p-x)*(p-y)*(p-z));
	   printf("x=%d,y=%d,z=%d,p=%f,area=%f\n",x,y,z,p,area);
       if((x+y<=z)||(x+z<=y)||(y+z<=x))
	   printf("你输入的无法构成三角形");
       else	
	   if(x==y&&y==z)	  
	   printf("这是一个等边三角形,其面积为%f\n",sqrt(p*(p-x)*(p-y)*(p-z)));	  
	   else 
	   if((x==y&&x!=z&&y!=z)||(x==z&&x!=y&&z!=y)||(z==y&&z!=x&&y!=x))
	   printf("这是一个等腰三角形,其面积为%f\n",sqrt(p*(p-x)*(p-y)*(p-z)));
	   else printf("这是一个普通的三角形，其面积为%f\n",sqrt(p*(p-x)*(p-y)*(p-z)));
	   
       
}
