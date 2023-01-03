/*Header file to hold the rules declaration and initialization*/


#define NH 0
#define NL 1
#define Z   2
#define PL 3
#define PH 4
#define FALSE 0
#define TRUE 1
#define PI 3.1415927

int val[5][3],rule[5][5];
int gr=0,md=0;
int inputgiven=FALSE;
int midpoint,endpoint,count;
float ang_int[2],vel_int[2];


struct info
{
	float mc,mp,l,ang,vp,vc,step,nang_a,max_acl,max_vel;
}in;

void init_rules()
{
	rule[0][0]=NH;  //IF angle is NH and ang. velocity is NH
	rule[0][1]=NH;	//IF angle is NL and ang. velocity is NH
	rule[0][2]=NH;	//IF angle is Z and ang. velocity is NH
	rule[0][3]=Z;	  //IF angle is PL and ang. velocity is NH
	rule[0][4]=PL;	 //IF angle is PH and ang. velocity is NH
	rule[1][0]=NH;	//IF angle is NH and ang. velocity is NL
	rule[1][1]=NH;	//IF angle is NL and ang. velocity is NL
	rule[1][2]=NL;	 //IF angle is Z and ang. velocity is NL
	rule[1][3]=Z;	  //IF angle is PL and ang. velocity is NL
	rule[1][4]=PH;	//IF angle is PH and ang. velocity is NL
	rule[2][0]=NH;	//IF angle is NH and ang. velocity is Z
	rule[2][1]=NL;	//IF angle is NL and ang. velocity is Z
	rule[2][2]=Z;	 //IF angle is Z and ang. velocity is Z
	rule[2][3]=PL;	//IF angle is PL and ang. velocity is Z
	rule[2][4]=PH;	//IF angle is PH and ang. velocity is Z	
	rule[3][0]=NH;	//IF angle is NH and ang. velocity is PL
	rule[3][1]=Z;	 //IF angle is NL and ang. velocity is PL
	rule[3][2]=PL;	//IF angle is Z and ang. velocity is PL
	rule[3][3]=PH;	//IF angle is PL and ang. velocity is PL
	rule[3][4]=PH;	//IF angle is PH and ang. velocity is PL
	rule[4][0]=NL;	 //IF angle is NH and ang. velocity is PH
	rule[4][1]=Z;	  //IF angle is NL and ang. velocity is PH
	rule[4][2]=PH;	//IF angle is Z and ang. velocity is PH
	rule[4][3]=PH;	//IF angle is PL and ang. velocity is PH
	rule[4][4]=PH;	//IF angle is PH and ang. velocity is PH
}

void init_val()
{
	
	val[0][0]=-10;
	val[0][1]=-4;
	val[0][2]=-2;
	
	val[1][0]=-4;
	val[1][1]=-2;
	val[1][2]=0;
	
	val[2][0]=-2;
	val[2][1]=0;
	val[2][2]=2;
	
	val[3][0]=0;
	val[3][1]=2;
	val[3][2]=4;
	
	val[4][0]=2;
	val[4][1]=4;
	val[4][2]=10;
			
}
	