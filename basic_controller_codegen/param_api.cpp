#include "param_api.h"
int get_param(char *p,float *value)
{

    char c[100];
    char param_name[20] = {0};
    float param_value=0;

    FILE *fptr = fopen("parameter.txt", "r");
    int flag = 0;
    while (fgets(c,sizeof(c),fptr) != NULL)
    {
        //printf("%s",c);
        if(sscanf(c,"%s = %f\n", param_name, &param_value) != 0)
        {

        }
        if(strcmp(param_name,p) == 0)
        {
            *value = param_value;
            flag = 1;
            break;
        }
    }
 
    fclose(fptr);
    return flag;
}