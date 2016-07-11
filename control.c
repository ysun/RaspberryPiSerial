#include <stdio.h>  
#include <unistd.h>  
 
union PARA
{
	char c[4];
	int d;
};
union CMD
{
	char c[4];
	int d;
	struct {
		char flag[2];
		char cmd;
		char rev;
	};
};
int main(int argc, char **argv)  
{  
	int ch;  
	opterr = 0;  
	while ((ch = getopt(argc, argv, "a:bcde")) != -1)  
	{  
		switch(ch)  
		{  
			case 'a':  
				break;	
			case 'b':  
				break;	
			case '?':
			default:
				printf("Here should print USAGE\n");  
		}  
		printf("opt [%c] with optopt: %s\n", ch, optarg);	
	}
	return 0;
}
