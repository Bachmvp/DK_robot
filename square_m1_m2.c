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
#define IR_SENS_LENGTH 6

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
        int w_cross;//true or false
	int fork; //true or false
	int fork_test;
    int ground;
	double ir_min;
	double irsensor[IR_SENS_LENGTH];
	double ir_dist[IR_SENS_LENGTH];
	/*
	double laser[10];
	int min_las;
	int min_las_2;
	int min_las_3;
	int min_las_r;
	int min_las_l;*/
} sensetype;

void update_sensors(sensetype *s, odotype *q);
const double linesensor_interp[2][8] = { {0.0217,0.0151,0.0140,0.0133,0.0131,0.0144,0.0157,0.0234},
				 {-1.1504,-0.8265,-0.7911,-0.7527, -0.7453,-0.8096,-0.9092,-1.3771} };

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

		//collision
		int flag_collision;
		
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
		double startpos;//unused
		double start_deg;
		double start_x;
		double start_y;
	       }motiontype;
	       
enum {mot_stop=1,mot_move,mot_turn};

void update_motcon(motiontype *p, odotype *q, sensetype *s);


int fwd(double dist, double speed,int time,int condition);
int turn(double angle, double speed,int time);
int follow_line(double speed, char dir, int flag_c, int time, int condition,int followdist);
int goto_box(double speed,double followdist);
int wait(double wait_time,int time);
void speed_control(int aim, motiontype *p, odotype *q, sensetype *s);
void speed_control_t(int aim, motiontype *p, odotype *q, sensetype *s);


typedef struct{
        int state,oldstate;
		int time;
		int sub_time;
		int sub_state;
	       }smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;
sensetype sens;

enum {
    ms_init,
    ms_fwd,
    ms_follow_line,
    ms_follow_white_line,
    ms_turn,
    ms_end,ms_goto_box,
    ms_measBox,
    ms_wait
};

const double BLACK_THRESHOLD = 0.15;
const double WHITE_THRESHOLD = 0.75;
const double robot_length = 0.25;//0.28;


int cross_counter;
int w_cross_counter;
//int m_cross_counter;

//logging test//
#define ARRAY_LENTGH 3000
static double record[3][ARRAY_LENTGH]={0};
static double record_odo[4][ARRAY_LENTGH]={0};
static double record_line[9][ARRAY_LENTGH]={0};
//end

int main()
{
  int running,n=0,arg,time=0,speed_go=0;
  double dist=0,angle=0,follow_dist = 0;
  double boxdist = 0;
  cross_counter = 0;
  w_cross_counter=0;
  //m_cross_counter=0;
  sens.cross = 0;
  sens.w_cross = 0;
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
     len=sprintf(buf,"scanpush cmd='zoneobst'\n");
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
  mot.flag_collision = 1;
  running=1; 
  mission.state=ms_init;
  mission.oldstate=-1;
  mission.sub_state = 0;
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
       n=4; dist=1;angle=90.0;
       mission.state= ms_measBox;
       speed_go = 20;
       follow_dist = 5;
     break;
  
     case ms_fwd:
       if (fwd(2,speed_go,mission.time,sens.cross))  mission.state=ms_follow_line;
     break;
     
     case ms_follow_line:
       if (follow_line(speed_go, 'l', 0, mission.time, sens.cross,follow_dist)) 
	mission.state = ms_end;
     break;
	 
     case ms_follow_white_line:
       if (follow_line(speed_go, 'c', 1, mission.time, sens.w_cross,follow_dist)) 
	mission.state = ms_end;
     break;
     
     case ms_measBox:
            if (follow_line(speed_go, 'l', 0, mission.time, sens.fork,follow_dist)) {
                
                boxdist = fabs(odo.x_pos) + 0.255 + laserpar[4];
                printf("Distance to box: %f\n", boxdist);
                printf("Distance to box: %f\n", laserpar[4]);
                mission.state = ms_end;
            }
            break;
     case ms_wait:
         if(wait(2, mission.time)){
           mission.state = ms_goto_box;}
           break;       
     case ms_goto_box:
         if (goto_box(speed_go,follow_dist)){
	    mission.state = ms_end;}
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
  printf("%f\n",odo.x_pos);
  //printf("%d,%f;%f;%d;%d;%f;%f;%d\n",mission.time,mot.speedcmd,mot.motorspeed_l,mission.state,mission.sub_state,mot.dist,mot.angle,sens.cross);
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot,&odo,&sens);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  if (time  % 100 ==0)
      //printf("%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",laserpar[0],laserpar[1],laserpar[2],laserpar[3],laserpar[4],laserpar[5],laserpar[6],laserpar[7],laserpar[8],laserpar[9]);
 time++;
      //printf("%d\n",sens.fork_test);
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
     mot.angle= angle*(M_PI/180);
     return 0;
   }
   else
     return mot.finished;
}

int wait(double wait_time, int time) {
    if (time < wait_time * 100) {
        return 0;
    }
    else {
        return 1;
    }
}


int follow_line(double speed, char dir, int flag_c, int time, int condition,int followdist) {
	int i = 0;
	int min = 0;
	int flag = 1;
	if (time == 0) {
		mot.cmd = mot_move;
		mot.speed_aim = speed;
		mot.dist = followdist;
		return 0;
	}
	else {
		if (condition)
		{
			mot.cmd = mot_stop;
                        //m_cross_counter++;
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

int goto_box(double speed,double followdist) {
	switch (mission.sub_state)
	{
	case 0:
		if (turn(25, speed, mission.sub_time)) {
			mission.sub_state = 1;
			mission.sub_time = (-1);
		}
		break;
        case 1:
		if (follow_line(speed, 'c', 0, mission.sub_time, 0, followdist)) {
			mission.sub_state = 2;
			mission.sub_time = (-1);
                        mot.flag_collision=0;
		}
		break;
	case 2:
		if (fwd(0.5, 20, mission.sub_time, 0)) {
			mission.sub_state = 3;
			mission.sub_time = (-1);
		}
		break;
	case 3:
		if (fwd(1.05, -20, mission.sub_time, 0)) {
			mission.sub_state = 4;
			mission.sub_time = (-1);
		}
		break;
        case 4:
		if (turn(180, speed, mission.sub_time)) {
			mission.sub_state = 5;
			mission.sub_time = (-1);
		}
		break;
        case 5:
		if (follow_line(speed, 'c', 0, mission.sub_time, sens.cross, followdist)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
                        mot.flag_collision=0;
		}
		break;
        case 6:
		if (turn(45, speed, mission.sub_time)) {
			mission.sub_state = 7;
			mission.sub_time = (-1);
		}
		break;
        case 7:
		if (follow_line(speed, 'c', 0, mission.sub_time, sens.cross, followdist)) {
			mission.sub_state = 8;
			mission.sub_time = (-1);
                        mot.flag_collision=0;
		}
		break;

        case 8:
		if (fwd(0.5, 20, mission.sub_time, 0)) {
			return 1;
		}
		break;
        }
	return 0;
}

int follow_white_line(double speed, double followdist) {
    switch (mission.sub_state)
    {   
    case 0:
        if (fwd(0.2, 20, mission.sub_time, 0)) {
            mission.sub_state = 1;
            mission.sub_time = (-1);
        }
        break;
    case 1:
        if (turn(85, speed, mission.sub_time){
            mission.sub_state = 2;
            mission.sub_time = (-1);
        }
        break;
    case 2:
        if (follow_line(speed, 'c', 1, mission.sub_time, sens.ground, followdist) {
            mission.sub_state = 3;
            mission.sub_time = (-1);
        }
        break;
    case 3:
        if (fwd(0.2, 20, mission.sub_time, 0)) {
            return 1;
        }
        break;

    }
    return 0;
}


void sm_update(smtype *p){
  if (p->state!=p->oldstate){
	   p->sub_state = 0;
	   p->time=0;
	   p->sub_time = 0;
       p->oldstate=p->state;
   }
   else {
     p->time++;
	 p->sub_time++;
   }
}

void speed_control(int aim, motiontype *p, odotype *q, sensetype *s)
{
	
	if (p->speed > aim) {
			p->speed-=1;}
	else if (p->speed < aim) {
			p->speed+=1;}
	if (s->ir_min < 0.20 && p->flag_collision){
		p->speed = 0;
                p->cmd=mot_stop;}
	p->speedcmd = p->speed / 127.0;
}

void speed_control_t(int aim, motiontype *p, odotype *q, sensetype *s)
{

	if (p->speed_t > aim) {
		p->speed_t-=1;
	}
	else if (p->speed_t < aim) {
		p->speed_t+=1;
	}
        p->speedcmd_t = p->speed_t / 127.0;
}

void update_sensors(sensetype *s, odotype *q)
{
	int i = 0;
	int min_line_sensor = 3;
	int max_line_sensor = 3;
	int fork_counter = 0;
	int line_counter = 0;
	int white_line_counter = 0;

	double Ka[5] = { 13.26, 18.12, 17.39, 15.00, 13.59 };
	double Kb[5] = { 82.15, 87.46, 83.09, 77.13, 61.65 };

	s->ir_min = 4;

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
            s->ground = 0;
			if (fork_counter == 0 || fork_counter == 2)
				fork_counter++;
		}
        else if ((s->linesensor[i] > BLACK_THRESHOLD) && (s->linesensor[i] < WHITE_THRESHOLD)) {
            s->ground = 1;
        }
		else {
			if (fork_counter == 1)
				fork_counter++;
			if (s->linesensor[i] > WHITE_THRESHOLD) {
                s->ground = 0;
				white_line_counter++;
			}
		}
	}

	if (line_counter == 8 && !s->cross) {
		cross_counter++;
		s->cross = 1;
	}
	if (white_line_counter == 8 && !s->cross) {
		w_cross_counter++;
		s->w_cross = 1;
	}
	if (line_counter < 8) {
		s->cross = 0;
	}
	if (white_line_counter < 8) {
		s->w_cross = 0;
	}
	if (fork_counter == 3)
	{
		s->fork_test++;
		if (s->fork_test > 2) {
			//printf("FORK found! \n");
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


	/**** IR sensors ****/
	for (i = 1; i < IR_SENS_LENGTH - 2; i++)
	{
		//Read in from sensor
		s->irsensor[i] = irsensor->data[i];

		s->ir_dist[i] = Ka[i] / (s->irsensor[i] - Kb[i]);

		//printf("%f \t", s->ir_dist[i]);

		if (i > 0 && i < IR_SENS_LENGTH - 2 && s->ir_dist[i]<s->ir_min && s->ir_dist[i]>0.05) {
			s->ir_min = s->ir_dist[i];
			//printf("Drove: %f m\n",q->x);
		}

	}

}


