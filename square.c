/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
//print//
#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
   double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

componentservertype lmssrv,camsrv;

 symTableElement * 
     getinputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('r'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }

    symTableElement * 
     getoutputref (const char *sym_name, symTableElement * tab)
     {
       int i;
       for (i=0; i< getSymbolTableSize('w'); i++)
         if (strcmp (tab[i].name,sym_name) == 0)
           return &tab[i];
       return 0;
     }
/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.06522	/* m */
#define WHEEL_SEPARATION 0.26	/* m */
#define E_d 1
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902 //24902
#define LINE_SENS_LENGTH 8

typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
	        //output signals
		double right_pos,left_pos;
		// internal variables
		int left_enc_old, right_enc_old;
		
		//new add//
		double center_deg,x_pos,y_pos;
		//new add end//
		} odotype;

void reset_odo(odotype *p);
void update_odo(odotype *p);


/*Sensor Control
*/
typedef struct {
	double linesensor[LINE_SENS_LENGTH];
	int min_line_sensor;
	int max_line_sensor;
	int black_line_follow;
	int lost; //true or false
	int cross; //true or false
	int fork; //true or false
	int fork_test;
	/*
	double prev_ir_r, prev_ir_l;
	double ir_min;
	double irsensor[IR_SENS_LENGTH];
	double ir_dist[IR_SENS_LENGTH];

	double laser[10];
	int min_las;
	int min_las_2;
	int min_las_3;
	int min_las_r;
	int min_las_l;*/
} sensetype;

void update_sensors(sensetype *s, odotype *q);
const double linesensor_interp[2][8] = { {0.0321,0.0252,0.0571,0.0452,0.0588,0.0374,0.0189,0.0617},
				 {-2.1908,-1.6003,-3.1495,-2.6528, -3.2212,-2.1868,-1.2967,-3.4303} };

/********************************************
* Motion control
*/

typedef struct{//input
                int cmd;
		int curcmd;
		//move speed
		double speedcmd;//current speed [-1,1]
		int speed; //[currenty spped[-128,127]
	    int speed_aim;//target speed[-128,127]
		//turn speed
		double speedcmd_t;//current speed [-1,1]
		int speed_t;//[currenty spped[-128,127]
		int speed_t_aim;//target speed[-128,127]

		
		double dist;
		double angle;
		double delta_v;
		double left_pos,right_pos;
		// parameters
		double w;
		double k_follow;
		//output
		double motorspeed_l,motorspeed_r; 
		int finished;
		// internal variables
		double startpos;
		double start_deg;
		double start_x;
		double start_y;
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p, odotype *q, sensetype *s);


int fwd(double dist, double speed,int time,int condition);
int turn(double angle, double speed,int time);
int follow_line(double speed, char dir, int flag_c, int time, int condition);
void speed_control(int aim, motiontype *p, odotype *q, sensetype *s);
void speed_control_t(int aim, motiontype *p, odotype *q, sensetype *s);


typedef struct{
                int state,oldstate;
		int time;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
sensetype sens;

enum {ms_init,ms_fwd,ms_follow_line,ms_turn,ms_end};

const double BLACK_THRESHOLD = 0.12;
const double WHITE_THRESHOLD = 0.75;
const double robot_length = 0.25;//0.28;


int cross_counter;

//logging test//
#define ARRAY_LENTGH 3000
static double record[3][ARRAY_LENTGH]={0};
static double record_odo[4][ARRAY_LENTGH]={0};
static double record_line[9][ARRAY_LENTGH]={0};
//end

int main()
{
  int running,n=0,arg,time=0,speed_go=0;
  double dist=0,angle=0;
  cross_counter = 0;
  sens.cross = 0;
  //open file//
  FILE* fp;
  FILE* fp2;
  FILE* fp3;
  fp=fopen("record_1.dat","w+");
  fp2=fopen("record_odo.dat","w+");
  fp3=fopen("record_line.dat","w+");
  int i_2=0;
  //end //

  /* Establish connection to robot sensors and actuators.
   */
     if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      } 
      
      printf("connected to robot \n");
      if ((inputtable=getSymbolTable('r'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      if ((outputtable=getSymbolTable('w'))== NULL){
         printf("Can't connect to rhd \n");
	 exit(EXIT_FAILURE); 
      }
      // connect to robot I/O variables
      lenc=getinputref("encl",inputtable);
      renc=getinputref("encr",inputtable);
      linesensor=getinputref("linesensor",inputtable);
      irsensor=getinputref("irsensor",inputtable);
           
      speedl=getoutputref("speedl",outputtable);
      speedr=getoutputref("speedr",outputtable);
      resetmotorr=getoutputref("resetmotorr",outputtable);
      resetmotorl=getoutputref("resetmotorl",outputtable);
     // **************************************************
//  Camera server code initialization
//

/* Create endpoint */
   lmssrv.port=24919;
   strcpy(lmssrv.host,"127.0.0.1");
   strcpy(lmssrv.name,"laserserver");
   lmssrv.status=1;
   camsrv.port=24920;
   strcpy(camsrv.host,"127.0.0.1");
   camsrv.config=1;
   strcpy(camsrv.name,"cameraserver");
   camsrv.status=1;

   if (camsrv.config) {
      int errno = 0; 
      camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( camsrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&camsrv);

   xmldata=xml_in_init(4096,32);
   printf(" camera server xml initialized \n");

}   
 
   
   
   
// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
   lmssrv.config=1;
   if (lmssrv.config) {
       char buf[256];
      int errno = 0,len; 
      lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
   if ( lmssrv.sockfd < 0 )
   {
    perror(strerror(errno));
    fprintf(stderr," Can not make  socket\n");
    exit(errno);
   }

   serverconnect(&lmssrv);
   if (lmssrv.connected){
     xmllaser=xml_in_init(4096,32);
     printf(" laserserver xml initialized \n");
     len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
     send(lmssrv.sockfd,buf,len,0);
   }

}   
   
 
  /* Read sensors and zero our position.
   */
  rhdSync();
  
  odo.w= WHEEL_SEPARATION;
  odo.cl=DELTA_M;
  odo.cr=odo.cl*E_d;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
  mot.k_follow = -0.15;
  mot.speed_aim = 0;
  mot.speed_t_aim = 0;
  mot.speed = 0;
  mot.speed_t = 0;
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;
  sens.black_line_follow = 1;
while (running){ 
   if (lmssrv.config && lmssrv.status && lmssrv.connected){
           while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
             xml_proca(xmllaser);
      }
      
      if (camsrv.config && camsrv.status && camsrv.connected){
          while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
             xml_proc(xmldata);
      }
       

  rhdSync();
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  update_sensors(&sens, &odo);
  update_odo(&odo);

 //write in array//
if(i_2<ARRAY_LENTGH){
record[0][i_2]=mission.time;
record[1][i_2]=mot.motorspeed_l;
record[2][i_2]=mot.motorspeed_r;
record_odo[0][i_2]=mission.time;
record_odo[1][i_2]=odo.x_pos;
record_odo[2][i_2]=odo.y_pos;
record_odo[3][i_2]=odo.center_deg;
record_line[0][i_2]=mission.time;
record_line[1][i_2]=linesensor->data[0];
record_line[2][i_2]=linesensor->data[1];
record_line[3][i_2]=linesensor->data[2];
record_line[4][i_2]=linesensor->data[3];
record_line[5][i_2]=linesensor->data[4];
record_line[6][i_2]=linesensor->data[5];
record_line[7][i_2]=linesensor->data[6];
record_line[8][i_2]=linesensor->data[7];
i_2++;
}
//end write
 
/****************************************
/ mission statemachine   
*/
   sm_update(&mission);
   
   switch (mission.state) {
     case ms_init:
       n=4; dist=1;angle=90.0/180*M_PI;
       mission.state= ms_follow_line;
       speed_go = 30;
     break;
  
     case ms_fwd:
       if (fwd(1,speed_go,mission.time,sens.cross))  mission.state=ms_follow_line;
     break;
     
     case ms_follow_line:
       if (follow_line(speed_go, 'c', 0, mission.time, sens.cross)) 
	mission.state = ms_end;
     break;
		 
     case ms_turn:
       if (turn(angle,speed_go,mission.time)){
         n=n-1;
	 if (n==0) 
	   mission.state=ms_end;
	 else
	   mission.state=ms_fwd;
	}
     break;  
     
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }  
/*  end of mission  */
  printf("%d,%f;%f;%d\n",mission.time,mot.speedcmd,mot.motorspeed_l,sens.cross);
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot,&odo,&sens);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  //if (time  % 100 ==0)
     //  printf(" laser %f \n",laserpar[3]);
 // time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;
    
}/* end of main control loop */


//write array in file
  int i_3=0;
  while(i_3<ARRAY_LENTGH){
  fprintf(fp,"%f;%f;%f\n",record[0][i_3],record[1][i_3],record[2][i_3]);
  fprintf(fp2,"%f;%f;%f;%f\n",record_odo[0][i_3],record_odo[1][i_3],record_odo[2][i_3],record_odo[3][i_3]);
  fprintf(fp3,"%f;%f;%f;%f;%f;%f;%f;%f;%f\n",record_line[0][i_3],record_line[1][i_3],record_line[2][i_3],record_line[3][i_3],record_line[4][i_3],record_line[5][i_3],record_line[6][i_3],record_line[7][i_3],record_line[8][i_3]);
  i_3++;}
  fclose(fp);
  fclose(fp2);
  fclose(fp3);
//end
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
 * Routines to convert encoder values to positions.
 * Encoder steps have to be converted to meters, and
 * roll-over has to be detected and corrected.
 */


void reset_odo(odotype * p)
{
  p->right_pos = p->left_pos = 0.0;
  p->right_enc_old = p->right_enc;
  p->left_enc_old = p->left_enc;
  //new add//
  p->center_deg=0.0;
  p->x_pos =p->y_pos=0.0;
  //new add end//
}

void update_odo(odotype *p)
{

  int delta;
  double delta_r_pos,delta_l_pos,delta_center_pos,delta_center_deg;

  delta = p->right_enc - p->right_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->right_enc_old = p->right_enc;
  delta_r_pos= delta * (p->cr);
  p->right_pos += delta_r_pos;
  
  delta = p->left_enc - p->left_enc_old;
  if (delta > 0x8000) delta -= 0x10000;
  else if (delta < -0x8000) delta += 0x10000;
  p->left_enc_old = p->left_enc;
  delta_l_pos= delta * (p->cl);
  p->left_pos += delta_l_pos;

  //new add//
  delta_center_pos=(delta_r_pos+delta_l_pos)/2;
  delta_center_deg=(delta_r_pos-delta_l_pos)/WHEEL_SEPARATION;
  p->center_deg=(p->center_deg)+delta_center_deg;
  p->x_pos=(p->x_pos)+(delta_center_pos)*cos(p->center_deg);
  p->y_pos=(p->y_pos)+(delta_center_pos)*sin(p->center_deg);
  //new add end//
}


void update_motcon(motiontype *p, odotype *q, sensetype *s) {

if (p->cmd !=0){
     
     p->finished=0;
     switch (p->cmd){
     case mot_stop:
       p->curcmd=mot_stop;
       break;
       case mot_move:
		   p->start_x = q->x_pos;
		   p->start_y = q->y_pos;
		   p->start_deg = q->center_deg;
       p->curcmd=mot_move;
       break;
       
       case mot_turn:
		   p->start_x = q->x_pos;
		   p->start_y = q->y_pos;
		   p->start_deg = q->center_deg;
         p->curcmd=mot_turn;
       break;
       
       
     }
     
     p->cmd=0;
   }
   
   switch (p->curcmd){
     case mot_stop:
		 //speed_control(p->speed_aim, p, q, s);
		 p->motorspeed_l = 0;
		 p->motorspeed_r = 0;
		 p->finished = 1;
     break;
     case mot_move:
       
     if (sqrt(pow((odo.x_pos) - (mot.start_x), 2) + pow((odo.y_pos) - (mot.start_y), 2)) > p->dist) {//(p->real_dist > p->dist){ when move is equal for fwd and follo wline
			 p->finished = 1;
			 p->motorspeed_l = 0;
			 p->motorspeed_r = 0;
		 }
	 else {

		 speed_control(p->speed_aim, p, q, s);

		 if (!(p->speedcmd + p->delta_v > 1 || p->speedcmd + p->delta_v < -1 || p->speedcmd - p->delta_v>1 || p->speedcmd - p->delta_v < -1))
		 {
			 p->motorspeed_l = p->speedcmd + p->delta_v;//*kfl/kf
			 p->motorspeed_r = p->speedcmd - p->delta_v;
		 }
		 else
		 {
			 printf("something went wrong, speed out of range");
			 p->cmd = mot_stop;
		 }

	 }
     break;
     
     case mot_turn:
		 speed_control_t(p->speed_t_aim, p, q, s);
		 if (p->angle > 0) {

			 if (q->center_deg - p->start_deg < p->angle) {
				 p->motorspeed_r = p->speedcmd_t;
				 p->motorspeed_l = -(p->speedcmd_t);
			 }
			 else {
				 p->motorspeed_r = 0;
				 p->motorspeed_l = 0;
				 p->finished = 1;
			 }
		 }
		 else {

			 if (q->center_deg - p->start_deg > p->angle) {
				 p->motorspeed_l = p->speedcmd_t;
				 p->motorspeed_r = -(p->speedcmd_t);
			 }
			 else {
				 p->motorspeed_r = 0;
				 p->motorspeed_l = 0;
				 p->finished = 1;
			 }
		 }
     break;
   }   
}   


int fwd(double dist, double speed,int time,int condition){
	mot.delta_v = 0;
   if (time==0){ 
     mot.cmd=mot_move;
     mot.speed_aim=speed;
     mot.dist=dist;
     return 0;
   }
   else
	 if (condition)
     mot.cmd = mot_stop;
     return mot.finished;
}

int turn(double angle, double speed,int time){
   if (time==0){ 
     mot.cmd=mot_turn;
     mot.speed_t_aim=speed;
     mot.angle=angle;
     return 0;
   }
   else
     return mot.finished;
}

int follow_line(double speed, char dir, int flag_c, int time, int condition) {
	int i = 0;
	int min = 0;
	int flag = 1;
	if (time == 0) {
		mot.cmd = mot_move;
		mot.speed_aim = speed;
		mot.dist = 1.3;
		return 0;
	}
	else {
		if (condition)
		{
			mot.cmd = mot_stop;
			return mot.finished;
		}
		if (sens.fork) {
			if (dir == 'r') {
				for (i = 0; i < LINE_SENS_LENGTH; i++) {
					if (sens.linesensor[i] < BLACK_THRESHOLD) {
						flag = 0;
						if (sens.linesensor[i] < sens.linesensor[min]) min = i;
					}
					//Break after first black line
					if (sens.linesensor[i] > BLACK_THRESHOLD && !flag) break;
				}
			}
			else if (dir == 'l') {
				min = 7;
				for (i = LINE_SENS_LENGTH - 1; i >= 0; i--) {
					if (sens.linesensor[i] < BLACK_THRESHOLD) {
						flag = 0;
						if (sens.linesensor[i] < sens.linesensor[min]) min = i;
					}
					//Break after sfirst black line from right
					if (sens.linesensor[i] > BLACK_THRESHOLD && !flag) break;
				}
			}
			else if (dir == 'c') {
				min = 3.5;
			}
			else {
				min = sens.min_line_sensor;
			}
		 sens.min_line_sensor = min;
		}
		mot.delta_v = mot.k_follow*(!flag_c*sens.min_line_sensor + flag_c * sens.max_line_sensor - 3.5)*mot.speedcmd;//follow black or white
	}

	return mot.finished;

}
void sm_update(smtype *p){
  if (p->state!=p->oldstate){
       p->time=0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
   }
}

void speed_control(int aim, motiontype *p, odotype *q, sensetype *s)
{
	
	if (p->speed > aim) {
			p->speed--;}
	else if (p->speed < aim) {
			p->speed++;}
	p->speedcmd = p->speed / 127.0;
}

void speed_control_t(int aim, motiontype *p, odotype *q, sensetype *s)
{

	if (p->speed_t > aim) {
		p->speed_t--;
	}
	else if (p->speed_t < aim) {
		p->speed_t++;
	}
        p->speedcmd = p->speed_t / 127.0;
}

void update_sensors(sensetype *s, odotype *q)
{
	int i = 0;
	int min_line_sensor = 3;
	int max_line_sensor = 3;
	int fork_counter = 0;
        int line_counter=0;
	/**** Line sensors ****/
	for (i = 0; i < LINE_SENS_LENGTH; i++)
	{
		//Read in from sensor and ajust with calibration values
		s->linesensor[i] = linesensor->data[i] * linesensor_interp[0][i] + linesensor_interp[1][i];

		if (s->linesensor[i] < s->linesensor[min_line_sensor]) {
			min_line_sensor = i;
		}
		if (s->linesensor[i] > s->linesensor[max_line_sensor]) {
			max_line_sensor = i;
		}
		if (s->linesensor[i] < BLACK_THRESHOLD) {
			line_counter++;
			if (fork_counter == 0 || fork_counter == 2)
			    fork_counter++;
		}
		else {
			if (fork_counter == 1)
				fork_counter++;
			}
	}

	if (line_counter == 8 && !s->cross) {
		cross_counter++;
		s->cross = 1;
	}

	if (line_counter < 8) {
		s->cross = 0;
	}

	if (fork_counter == 3)
	{
		s->fork_test++;
		if (s->fork_test > 2) {
			printf("FORK found! \n");
			s->fork = 1;
		}

		else
		{
			s->fork = 0;
		}
	}
	else
	{
		s->fork_test = 0;
		s->fork = 0;
	}
	s->min_line_sensor = min_line_sensor;
	s->max_line_sensor = max_line_sensor;
}


