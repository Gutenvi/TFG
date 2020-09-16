#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


int hex2int(char *hex) {
    uint32_t val = 0;
    while (*hex) {
        // get current character then increment
        uint8_t byte = *hex++; 
        // transform hex character to the 4bit equivalent number, using the ascii table indexes
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
        // shift 4 to make space for new digit, and add the 4 bits of the new digit 
        val = (val << 4) | (byte & 0xF);
    }
    return (int)val;
}
//int option0,option1,option2,option3,option4,option5;
int options[6] = {0,0,0,0,0,0};
char **opciones = {"RIGHT HIP ANGLE; ", "RIGHT KNEE ANGLE; ", "RIGHT ANKLE ANGLE; ", "LEFT HIP ANGLE; ", "LEFT KNEE ANGLE; ", "LEFT ANKLE ANGLE", "RIGHT HIP GAUGE TORQUE; ", "RIGHT KNEE GAUGE TORQUE; ","RIGHT ANKLE GAUGE TORQUE; ","LEFT HIP GAUGE TORQUE; ","LEFT KNEE GAUGE TORQUE; ", "LEFT ANKLE GAUGE TORQUE", "RIGHT HEEL FSR; ", "RIGHT TOE FSR; ","LEFT HEEL FSR; ","LEFT TOE FSR; ", "BATTERY VOLTAGE; ", "FUTURE USE", "RIGHT HIP MOTOR TORQUE; ", "RIGHT KNEE MOTOR TORQUE; ", "RIGHT ANKLE MOTOR TORQUE; ", "LEFT HIP MOTOR TORQUE; ", "LEFT KNEE MOTOR TORQUE; ", "LEFT ANKLE MOTOR TORQUE"};

int main(int argc, char **argv){
	FILE *fp;
	//printf("%s\n", argv[1] = file , argv[2] = dir, argv[3] = id );
	fp = fopen(argv[1], "r"); // read mode
 	int linenum = 0;
  FILE *printfile;
  char* _dir = malloc(sizeof(char)*180);
  //printf("%i\n%s//%s//%s//\n",argc, argv[1],argv[2],argv[3] );
  //fflush(stdout);

  switch(argv[3][1]){
      case '1':
        sprintf(_dir,"%s/parsed_candump_110.csv",argv[2]);
        printfile = fopen(_dir,"w");
        fprintf(printfile, "TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; %s%s%s%s%s%s\n", "RIGHT HIP ANGLE; ", "RIGHT KNEE ANGLE; ", "RIGHT ANKLE ANGLE; ", "LEFT HIP ANGLE; ", "LEFT KNEE ANGLE; ", "LEFT ANKLE ANGLE");
        break;
      case '2':
        sprintf(_dir,"%s/parsed_candump_120.csv",argv[2]);
        printfile = fopen(_dir,"w");
        fprintf(printfile, "TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; %s%s%s%s%s%s\n","RIGHT HIP GAUGE TORQUE; ", "RIGHT KNEE GAUGE TORQUE; ","RIGHT ANKLE GAUGE TORQUE; ","LEFT HIP GAUGE TORQUE; ","LEFT KNEE GAUGE TORQUE; ", "LEFT ANKLE GAUGE TORQUE");
        break;
      case '3':
        sprintf(_dir,"%s/parsed_candump_130.csv",argv[2]);
        printfile = fopen(_dir,"w");
        fprintf(printfile, "TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; %s%s%s%s%s%s\n", "RIGHT HEEL FSR; ", "RIGHT TOE FSR; ","LEFT HEEL FSR; ","LEFT TOE FSR; ", "BATTERY VOLTAGE; ", "FUTURE USE");
        break;
      case '4':
        sprintf(_dir,"%s/parsed_candump_140.csv",argv[2]);
        printfile = fopen(_dir,"w");
        fprintf(printfile, "TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; %s%s%s%s%s%s\n", "RIGHT HIP MOTOR TORQUE; ", "RIGHT KNEE MOTOR TORQUE; ", "RIGHT ANKLE MOTOR TORQUE; ", "LEFT HIP MOTOR TORQUE; ", "LEFT KNEE MOTOR TORQUE; ", "LEFT ANKLE MOTOR TORQUE");
        break;
      case '5':
        sprintf(_dir,"%s/parsed_candump.csv",argv[2]);
        printfile = fopen(_dir,"w");
        fprintf(printfile, "TIMESTAMP; DATE; ACTUAL TIME; CURRENT TIME; ERROR; SAMPLE; ID; %s%s%s%s%s%s\n", "BYTE_1; ", "BYTE_2; ", "BYTE_3; ", "BYTE_4; ", "BYTE_5; ", "BYTE_6");
        break;
  }

   if (fp == NULL)
   {
      perror("Error while opening the file.\n");
      exit(EXIT_FAILURE);
   }
 
   //printf("The contents of %s file are:\n", file_name);
   char line[256];
   while(fgets(line, 256, fp) != NULL)
	{

		//printf("%s\n", line );
		//fflush(stdout);
        char timestamp[256],actual[256],actual_2[256], current[256],error[256],sample[256],id[256],data[256];

        linenum++;
        if(line[0] == 'T') continue;

        if(sscanf(line, "%s %s %s %s %s %s %s %s", timestamp, actual, actual_2, current, error, sample, id, data) != 8)
        {
                //fprintf(stderr, "Syntax error, line %d\n", linenum);
                //continue;
        }

        char *string = malloc(sizeof(char)*3);
       
            
            for(int i = 0; i < 6; i++){
              string[0] = data[0+2*i];
              string[1] = data[1+2*i];
              options[i] = hex2int(string);
            }
            fprintf(printfile, "%s %s %s %s %s %s",timestamp, actual, actual_2, current, error, sample);
            if(argv[3][1] == '5')
              fprintf(printfile, " %s", id );
            fprintf(printfile," %i; %i; %i; %i; %i; %i\n", options[0],options[1],options[2],options[3],options[4],options[5]);
            
        
        //printf("%s/%s/%s/%s/%s/%s/%s\n", actual, actual_2, current, error, sample, id, data);
        //fflush(stdout);
}
}