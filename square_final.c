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
//smr12
const double linesensor_interp[2][8] = { {0.0249,0.0178,0.0165,0.0151,0.0144,0.0148,0.0159,0.0246},
				     {-1.3223,-0.9732,-0.9348,-0.8548, -0.8204,-0.8336,-0.9237,-1.4448} };
//const double linesensor_interp[2][8] = { {0.0419,0.0479,0.0329,0.0401,0.0272,0.0320,0.0415,0.0518},
				 //{-1.8455,-2.1739,-1.4548,-1.7778,-1.2169,-1.4192,-1.8654,-2.3161} };
//smr4
//const double linesensor_interp[2][8] = { {0.0316,0.0273,0.0258,0.0242,0.0250,0.0301,0.0276,0.0328},
				 //{-1.6655,-1.4252,-1.4798,-1.3452,-1.3824,-1.6428,-1.5653,-1.8712} };
//smr2
//const double linesensor_interp[2][8] = { {0.0870,0.0738,0.0725,0.0784,0.0803,0.0816,0.0782,0.0876},
				 //{-4.6040,-3.9232,-3.8695,-4.1771,-4.2640,-4.2602,-4.1728,-4.7049} };
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
                double k_wall;
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
int follow_wall(double speed, char dir, double dist_wall, int time, int condition);
int goto_box(double speed,double followdist);
int gate_loose(double speed, int time,double followdist);
int gate_wall(double speed, double dist_wall, int time);
int wait(double wait_time,int time);
int follow_white_line(double speed,double followdist);
int garage_into(double speed,double followdist);
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

enum {ms_init,ms_fwd,ms_follow_line,ms_follow_white_line,ms_turn,ms_end,ms_goto_box,ms_measBox,ms_wait,ms_gate_loose,ms_gate_wall,ms_wait_2,ms_garage_into,ms_wait_3,ms_follow_wall};

const double BLACK_THRESHOLD = 0.13;
const double WHITE_THRESHOLD = 0.75;
const double robot_length = 0.25;//0.28;
const double back_gate_dist=0.025;
const double back_gate_w_dist = 0.2;


int cross_counter;
int w_cross_counter;
double dist_to_wall = 1.3;
//double dist_to_white_gate_1 = 0;
//double dist_to_white_gate_2 = 0;
//double wg_y_1 = 0;
//double wg_y_2 = 0;
//double gw = 0;
//double white_line_turn = 0;
//double garage_w_1 = 0;
//double garage_w_2 = 0;

//int m_cross_counter;

//logging test//
#define ARRAY_LENTGH 3000
static double record[3][ARRAY_LENTGH]={0};
static double record_odo[4][ARRAY_LENTGH]={0};
static double record_line[9][ARRAY_LENTGH]={0};
//end
double white_pos=0;
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
  mot.k_follow = -0.35;//-0.3
  mot.k_wall=4;
  mot.speed_aim = 0;
  mot.speed_t_aim = 0;
  mot.speed = 0;
  mot.speed_t = 0;
  mot.flag_collision = 0;
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
record_line[1][i_2]=laserpar[0];
record_line[2][i_2]=laserpar[1];
record_line[3][i_2]=laserpar[2];
record_line[4][i_2]=laserpar[3];
record_line[5][i_2]=laserpar[4];
record_line[6][i_2]=laserpar[5];
record_line[7][i_2]=laserpar[7];
record_line[8][i_2]=laserpar[8];
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
       speed_go = 35;
       follow_dist = 5;
     break;
  
     case ms_fwd:
       if (fwd(2,speed_go,mission.time,sens.cross))  mission.state=ms_follow_line;
     break;
     
     case ms_follow_line:
       if (follow_line(speed_go, 'c', 1, mission.time,odo.x_pos,5)) 
	mission.state = ms_end;
     break;
	 
     case ms_follow_white_line:
       if (follow_white_line(speed_go,follow_dist)) 
	mission.state = ms_garage_into;
     break;
     case ms_follow_wall:
       if (follow_wall(speed_go/2, 'l',0.2, mission.time, 0)){
         mission.state= ms_follow_white_line;}
     break;
     case ms_measBox:
            if (follow_line(speed_go/2, 'c', 0, mission.time, sens.fork,follow_dist)){ 
                //if(turn(-20,speed_go,mission.time)){
                boxdist = fabs(odo.y_pos) + 0.255 + laserpar[4];
                printf("Distance to box: %f\n", boxdist);
              //  printf("Distance to box: %f\n", laserpar[4]);
                mission.state = ms_wait;
            }
            break;
     case ms_wait:
         if(wait(2, mission.time)){
           mission.state = ms_goto_box;}
           break;
     case ms_wait_2:
         if(wait(2, mission.time)){
           mission.state = ms_follow_line;}
           break; 
     case ms_wait_3:
         if(wait(2, mission.time)){
           mission.state = ms_garage_into;}
           break;                  
     case ms_goto_box:
         if (goto_box(speed_go,follow_dist)){
	    mission.state = ms_gate_loose;}
	 break;

     case ms_gate_loose:
         if (gate_loose(speed_go,mission.time,follow_dist)){
	    mission.state = ms_gate_wall;}
	 break;

	 case ms_gate_wall:
		 if (gate_wall(speed_go, 0.3, mission.time)) {
			 mission.state = ms_follow_white_line;
		 }
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
     case ms_garage_into:
          if (garage_into(speed_go,follow_dist)){
             mission.state = ms_end;}
     break;
     case ms_end:
       mot.cmd=mot_stop;
       running=0;
     break;
   }  
/*  end of mission  */
  //printf("%d,%f;%f;%d;%d;%f;%f;%d\n",mission.time,mot.speedcmd,mot.motorspeed_l,mission.state,mission.sub_state,mot.dist,mot.angle,sens.cross);
 //printf("%d;%f;%f\n",mission.sub_time,odo.x_pos,odo.y_pos);
  mot.left_pos=odo.left_pos;
  mot.right_pos=odo.right_pos;
  update_motcon(&mot,&odo,&sens);
  speedl->data[0]=100*mot.motorspeed_l;
  speedl->updated=1;
  speedr->data[0]=100*mot.motorspeed_r;
  speedr->updated=1;
  //if (time  % 100 ==0)
 // printf("%f;%f;%f;%f;%f;%f\n",sens.ir_dist[0],sens.ir_dist[1],sens.ir_dist[2],sens.ir_dist[3],sens.ir_dist[4],sens.ir_dist[5]);
     //  printf(" laser %f \n",laserpar[3]);
   //printf("%f;%f;%d;%d\n",laserpar[0],laserpar[8],mission.sub_state,sens.fork);
    time++;
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

//int measBox(double speed,int time){



//}

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


int follow_wall(double speed, char dir, double dist_wall, int time, int condition) {
	double laser_wall = 0;
	double dist = 0;
	if (dir == 'r') {
		if (laserpar[7] > dist_wall + 0.3 && laserpar[8] > dist_wall + 0.2) {
			printf("no wall to follow;%f;%f\n",laserpar[7],laserpar[8]);
			return 1;
		}
		else {
			laser_wall = laserpar[8];
		}
	}
	else if (dir == 'l') {
		if (laserpar[1] > dist_wall + 0.3 && laserpar[0] > dist_wall + 0.2) {
			printf("no wall to follow");
			return 1;
		}
		else {
			laser_wall = laserpar[0];
		}

	}
	dist = laser_wall - dist_wall;
	if (dist > 0.15) dist = 0.15;
	if (dist < -0.15) dist = -0.15;
        if((dist>-0.04)&&(dist<0.04)) dist=0;
	mot.delta_v = (dist)*mot.k_wall * mot.speedcmd; //mot.k_wall
	//printf("ir4: %f, delta_v: %f \n", sens.ir_dist[4], mot.delta_v);

	if (condition)
	{
		mot.cmd = mot_stop;
		return 1;
	}
	if (time == 0) {
		mot.cmd = mot_move;
		mot.speed_aim = speed;
		mot.dist = 2;
		return 0;
	}
	else if (mot.finished == 1)
		return mot.finished;
	else
		return 0;
}
int follow_white_line(double speed, double followdist){
switch (mission.sub_state)
{    case 0:
		if (follow_line(speed, 'c', 0, mission.sub_time, sens.cross, followdist)) {
			mission.sub_state = 4;
			mission.sub_time = (-1);
                        
		}
		break;
     case 4:
          if(fwd(0.15,speed/2,mission.sub_time,0)){
            mission.sub_state = 5;
            mission.sub_time = (-1); }
      break;
     case 5:
		if (turn(70, speed, mission.sub_time)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
		}
		break;
     case 6:
          if(fwd(0.2,speed/2,mission.sub_time,0)){
            white_pos=odo.y_pos;
            mission.sub_state = 1;
            mission.sub_time = (-1); }
      break;
     case 1:
         if (follow_line(speed/1.5,'c',1,mission.sub_time,fabs(odo.y_pos-white_pos)>2.2/*(sens.linesensor[3]<BLACK_THRESHOLD)&&(sens.linesensor[4]<BLACK_THRESHOLD)*/,5)){
        // dist_to_white_gate_1 = laserpar[0];
         //wg_y_1 = odo.y_pos;
         //printf("dist_to_white gate_1 %f odo.y = %f\n",dist_to_white_gate_1,wg_y_1);
         mission.sub_state = 3;
         mission.sub_time = (-1);
         }
     break;
     case 3:
         if (fwd(0.4,speed/2,mission.sub_time,0)){
         mission.sub_state = 2;
         mission.sub_time = (-1);}
     break;
     case 2:
         if (turn(50, speed, mission.sub_time)) {
			mission.sub_state = 9;
         mission.sub_time = (-1);
		}
		break;
     case 9:
         if (fwd(0.25,speed/2,mission.sub_time,0)){
         return 1;}
     break;
      /*case 9:
                if (fwd(0.45, speed/2, mission.sub_time, 0)) {//back up to center in the gate.
			mission.sub_state = 2;
			mission.sub_time = (-1);
		}
		break;*/
     //case 2:
		 //if (follow_line(speed / 4, 'c', 0, mission.sub_time,laserpar[4] < 0.2, followdist)) {
			 //  dist_to_white_gate_2 = laserpar[0];
			 //  wg_y_2 = odo.y_pos;
			 //  gw = fabs(wg_y_2 -  wg_y_1); 
			 //  printf("dist_to_white gate_2 %f odo.y = %f gw = %f\n",dist_to_white_gate_2,wg_y_2,gw);
			 //  white_line_turn  = (M_PI/2 - atan((dist_to_white_gate_1 - dist_to_white_gate_2)/gw))/(M_PI/180);
			 //  printf("white_line_turn %f\n",white_line_turn);
			 //return 1;
		// }
     //break;
    /* case 3:
         if(fwd(back_gate_w_dist,-1*(speed/4),mission.sub_time,0)){
         mission.sub_state = 4;
         mission.sub_time = (-1);}
     break;
     case 4:
         if (turn(white_line_turn + 45, speed/3, mission.sub_time)){
            mission.sub_state = 5;
           mission.sub_time = (-1);}
     break;
     case 5:
          if (fwd(0.2,speed/3,mission.sub_time,0)){
             mission.sub_state = 6;
             mission.sub_time = (-1);}
     break;
     case 6:
         if (follow_line(speed/3,'c',1,mission.sub_time,0,followdist)){
             mission.sub_state = 7;
             mission.sub_time = (-1);}
     break;
     case 7:
          if (fwd(0.01,speed/3,mission.sub_time,0)){
           return 1;} */
}
            return 0;     
}


int goto_box(double speed,double followdist) {
	switch (mission.sub_state)
	{
	case 0:
		if (turn(20, speed, mission.sub_time)) {
			mission.sub_state = 1;
			mission.sub_time = (-1);
		}
		break;
        case 1:
		if (follow_line(speed/2, 'c', 0, mission.sub_time, (laserpar[4]<0.2)&&(laserpar[4]!=0), followdist)) {
			mission.sub_state = 2;
			mission.sub_time = (-1);
                       // return 1;
                        mot.flag_collision=0;
                        
		}
		break;
	case 2:
		if (fwd(0.6, 20, mission.sub_time, 0)) {
			mission.sub_state = 3;
			mission.sub_time = (-1);
		}
		break;
	case 3:
		if (fwd(0.7, -20, mission.sub_time, 0)) {
			mission.sub_state = 4;
			mission.sub_time = (-1);
		}
		break;
        case 4:
		if (turn(160, speed, mission.sub_time)) {
			mission.sub_state = 5;
			mission.sub_time = (-1);
		}
		break;
        case 5:
		if (follow_line(speed/1.5, 'c', 0, mission.sub_time, sens.cross, followdist)) {
			mission.sub_state = 9;
			mission.sub_time = (-1);
                        mot.flag_collision=0;
		}
		break;
        case 9:
		if (fwd(0.1, 20, mission.sub_time, 0)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
		}
		break;
        case 6:
		if (turn(70, speed, mission.sub_time)) {
			mission.sub_state = 10;
			mission.sub_time = (-1);
		}
		break;
        case 10:
		if (fwd(0.1, 20, mission.sub_time, 0)) {
			mission.sub_state = 7;
			mission.sub_time = (-1);
		}
		break;
        case 7:
		if (follow_line(speed/1.5, 'c', 0, mission.sub_time, sens.cross, followdist)) {
			mission.sub_state = 8;
			mission.sub_time = (-1);
            mot.flag_collision=0;
		}
		break;

        case 8:
		if (fwd(0.2, 20, mission.sub_time, 0)) {
			return 1;
		}
		break;
        }
	return 0;
}

int gate_loose(double speed, int time,double followdist) {
	switch (mission.sub_state)
	{
	case 0:
		if (follow_line(speed / 2, 'c', 0, mission.sub_time, sens.cross,followdist)) {
			mission.sub_state = 1;
			mission.sub_time = (-1);
			dist_to_wall = 1.4;
			mot.flag_collision=0;
		}
		break;
	case 1:
		if (follow_line(speed / 2, 'c', 0, mission.sub_time, (laserpar[8] < (dist_to_wall-0.3))&&(laserpar[8]!=0),followdist)) {
			printf("Gate found maybe at laser: %f?\n", laserpar[8]);
			mission.sub_state = 9;
			mission.sub_time = (-1);
		}
		break;
        case 9:
                if (fwd(0.4, speed/2, mission.sub_time, 0)) {//back up to center in the gate.
			mission.sub_state = 2;
			mission.sub_time = (-1);
		}
		break;
	case 2:
		if (follow_line(speed / 3, 'c', 0, mission.sub_time, (laserpar[8] >(dist_to_wall-0.3))&&(laserpar[8]!=0),followdist)) {//go forward no longer see gatepost
			printf("No more gate at laser: %f\n", laserpar[8]);
                        dist_to_wall=laserpar[8];
			mission.sub_state = 3;
			mission.sub_time = (-1);
		}
		break;
	case 3:
		if (follow_line(speed / 3, 'c', 0, mission.sub_time, (laserpar[8] < (dist_to_wall-0.3))&&(laserpar[8]!=0),followdist)) {//go forward see gatepost
			printf("gate!! %f\n", laserpar[8]);
			mission.sub_state = 4;
			mission.sub_time = (-1);
		}
		break;
        case 4:
		if (follow_line(speed / 3, 'c', 0, mission.sub_time, (laserpar[8] >(dist_to_wall-0.3))&&(laserpar[8]!=0),followdist)) {//go forward no longer see gatepost
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 5;
			mission.sub_time = (-1);
		}
		break;
	case 5:
		if (fwd(back_gate_dist-0.01, -1 * (speed/2), mission.sub_time, 0)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
		}
		break;
	case 6:
		if (turn(-75, speed / 2, mission.sub_time)) {
			mission.sub_state = 7;
			mission.sub_time = (-1);
		}
		break;
	case 7:
		if (fwd(dist_to_wall-0.1-robot_length, speed/2, mission.sub_time, 0)) {//go to 20 cm from wall
			mission.sub_state = 8;
			mission.sub_time = (-1);
		}
		break;
	case 8:
		if (turn(70, speed / 2, mission.sub_time)) {
			return 1;
		}
		break;
	}
	return 0;
}


int gate_wall(double speed,double dist_wall, int time) {
	speed = speed / 2;
	switch (mission.sub_state)
	{
	case 0:
		if (follow_wall(speed, 'r', dist_wall, mission.sub_time, (laserpar[8] > (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 2;
			mission.sub_time = (-1);
		}
		break;
        /*case 1:
		if (fwd(0.4, speed, mission.sub_time, 0)) {
			
			mission.sub_state = 2;
			mission.sub_time = (-1);
		}
		break;*/
	case 2:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] < (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("Gate found maybe at laser: %f?\n", laserpar[8]);
			mission.sub_state = 3;
			mission.sub_time = (-1);
		}
		break;
        
	case 3:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] > (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 4;
			mission.sub_time = (-1);
		}
		break;
	case 4:
		if (fwd(back_gate_dist, -1 * speed, mission.sub_time, 0)) {//back up to center in the gate.
			mission.sub_state = 5;
			mission.sub_time = (-1);
		}
		break;
	case 5:
		if (turn(-75, speed, mission.sub_time)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
		}
		break;
	case 6:
		if (fwd(2*dist_wall+0.04,speed, mission.sub_time, 0)) {
			mission.sub_state = 7;
			mission.sub_time = (-1);
		}
		break;
	case 7:
		if (turn(-80, speed, mission.sub_time)) {
			mission.sub_state = 8;
			mission.sub_time = (-1);
		}
		break;
	case 8:
		if (fwd(0.4, speed, mission.sub_time, 0)) {
			mission.sub_state = 9;
			mission.sub_time = (-1);
		}
		break;
	case 9:
		if (follow_wall(speed, 'r', dist_wall, mission.sub_time, (laserpar[8] > (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 11;
			mission.sub_time = (-1);
		}
		break;
	/*case 10:
		if (fwd(1, speed, mission.sub_time, laserpar[8] > dist_wall)) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 11;
			mission.sub_time = (-1);
		}
		break;*/
	case 11:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] < (dist_wall+0.2))&&(laserpar[8]!=0))) {
			printf("Gate found maybe at laser: %f?\n", laserpar[8]);
			mission.sub_state = 12;
			mission.sub_time = (-1);
		}
		break;
        
	case 12:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] > (dist_wall+0.2))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 13;
			mission.sub_time = (-1);
		}
		break;
	case 13:
		if (fwd(back_gate_dist, -1 * speed, mission.sub_time, 0)) {//back up to center in the gate.
			mission.sub_state = 14;
			mission.sub_time = (-1);
		}
		break;
	case 14:
		if (turn(-75, speed, mission.sub_time)) {
			mission.sub_state = 15;
			mission.sub_time = (-1);
		}
		break;
	case 15:
		if (fwd(2*dist_wall+0.04, speed, mission.sub_time, 0)) {
			mission.sub_state = 16;
			mission.sub_time = (-1);
		}
		break;
	case 16:
		if (turn(-80, speed, mission.sub_time)) {
			mission.sub_state = 17;
			mission.sub_time = (-1);
		}
		break;
	case 17:
		if (fwd(0.4, speed, mission.sub_time, 0)) {
			mission.sub_state = 18;
			mission.sub_time = (-1);
		}
		break;
	case 18:
		if (follow_wall(speed, 'r', dist_wall, mission.sub_time, (laserpar[8] > (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 20;
			mission.sub_time = (-1);
		}
		break;
	/*case 19:
		if (fwd(1, speed, mission.sub_time, laserpar[0] > dist_wall)) {
			printf("No more gate at laser: %f\n", laserpar[0]);
			mission.sub_state = 20;
			mission.sub_time = (-1);
		}
		break;*/
	case 20:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] < (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("Gate found maybe at laser: %f?\n", laserpar[8]);
			mission.sub_state = 21;
			mission.sub_time = (-1);
		}
		break;
	case 21:
		if (fwd(1, speed, mission.sub_time, (laserpar[8] > (dist_wall+0.4))&&(laserpar[8]!=0))) {
			printf("No more gate at laser: %f\n", laserpar[8]);
			mission.sub_state = 22;
			mission.sub_time = (-1);
		}
		break;
	case 22:
		if (fwd(back_gate_dist, -1 * speed, mission.sub_time, 0)) {//back up to center in the gate.
			mission.sub_state = 23;
			mission.sub_time = (-1);
		}
		break;
	case 23:
		if (turn(-75, speed, mission.sub_time)) {
			mission.sub_state = 24;
			mission.sub_time = (-1);
                        
		}
		break;
	case 24:
		if (follow_line(speed, 'c', 0, mission.sub_time, sens.cross, 1)) {
                        mission.sub_state = 25;
			mission.sub_time = (-1);
			
		}
		break;
        case 25:
		if (turn(90, speed, mission.sub_time)) {
			return 1;
                        
		}
		break;
	}
	return 0;
}
int garage_into(double speed,double followdist){
	switch (mission.sub_state)
	{

	case 0:
		if (follow_line(speed, 'c', 0, mission.sub_time,(laserpar[4] < 0.2)&&(laserpar[4]!=0),followdist)) {
			mission.sub_state = 1;
			mission.sub_time = (-1);
		}
		break;
	case 1:
		if (turn(80, speed / 2, mission.sub_time)) {
			mission.sub_state = 3;
			mission.sub_time = (-1);
		}
		break;
	case 3:
		if (fwd(0.43 + robot_length, speed, mission.sub_time, 0)) {
			mission.sub_state = 4;
			mission.sub_time = (-1);
		}
		break;
	case 4:
		if (turn(-80, speed / 2, mission.sub_time)) {
			mission.sub_state = 5;
			mission.sub_time = (-1);
		}
		break;
	case 5:
		if (fwd(0.30 + robot_length, speed, mission.sub_time, 0)) {
			mission.sub_state = 6;
			mission.sub_time = (-1);
		}
		break;
	case 6:
		if (turn(-165, speed * 2, mission.sub_time)) {//150
			mission.sub_state = 7;
			mission.sub_time = (-1);
		}
		break;
	case 7:
		if (fwd(0.4, speed, mission.sub_time, 0)) {
			mission.sub_state = 8;
			mission.sub_time = (-1);
		}
		break;
	case 8:
		if (turn(70, speed / 2, mission.sub_time)) {
			mission.sub_state = 9;
			mission.sub_time = (-1);
		}
		break;
	case 9:
		if (fwd(0.4, speed, mission.sub_time, sens.cross)) {
			mission.sub_state = 10;
			mission.sub_time = (-1);
		}
		break;
	case 10:
		if (fwd(0.1, speed, mission.sub_time, 0)) {
			mission.sub_state = 11;
			mission.sub_time = (-1);
		}
		break;
	case 11:
		if (turn(90, speed / 2, mission.sub_time)) {
			mission.sub_state = 12;
			mission.sub_time = (-1);
		}
		break;
	case 12:
		if (follow_line(speed, 'c', 0, mission.sub_time,laserpar[4] < 0.1,followdist)) {
			return 1;
		}
		break;

	}
return 0;
}//function
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

	//double Ka[5] = { 18.40, 15.35, 15.88, 14.28, 14.46 };
	//double Kb[5] = { 55.52, 43.30, 70.64, 62.26, 82.47 };
          double Ka[5] = {13.26, 18.12, 17.39, 15.00 , 13.19};
	  double Kb[5] = { 82.15,87.46,83.09,77.13,61.65};

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
			if (fork_counter == 0 || fork_counter == 2)
				fork_counter++;
		}
		else {
			if (fork_counter == 1)
				fork_counter++;
			if (s->linesensor[i] > WHITE_THRESHOLD) {
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
	for (i = 1; i < IR_SENS_LENGTH-2; i++)
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


