/* -----------------------------------------------------------------------------------------------------------------------------------
Program to visualize the performance of the Fuzzy Logic Based vehicle-controller to balance
													Inverted Pendulum 
--------------------------------------------------------------------------------------------------------------------------------------

Date started: 14.10.2002													Date Completed: 

										Made by: Prashant Ganesh

----------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------*/

#include<afxwin.h>
#include<math.h>
#include"resource.h"
#include"first.h"

// Class for the about dialog box
class aboutbox:public CDialog
{
public:
	aboutbox(int n):CDialog(n)
	{
	}
};

// Class for the help dialog box
class helpbox:public CDialog
{
public:
	helpbox(int n):CDialog(n)
	{
	}
};

// Class for the input dialog box
class inputbox:public CDialog
{
public:
	inputbox(int n):CDialog(n)
	{
	}

	int OnInitDialog()
	{
		CDialog::OnInitDialog();
		SetDlgItemText(IDC_EDIT1,"1");
		SetDlgItemText(IDC_EDIT2,"0.1");
		SetDlgItemText(IDC_EDIT3,"1");
		SetDlgItemText(IDC_EDIT4,"5");
		SetDlgItemText(IDC_EDIT5,"0");
		SetDlgItemText(IDC_EDIT6,"0");
		SetDlgItemText(IDC_EDIT7,"10");
		SetDlgItemText(IDC_EDIT8,"40");
		return TRUE;
	}

	void DoDataExchange(CDataExchange *pdx)
	{
		CDialog::DoDataExchange(pdx);

		DDX_Text(pdx,IDC_EDIT1,in.mc);
		DDV_MinMaxFloat(pdx,in.mc,0.00f,20.00f);

		DDX_Text(pdx,IDC_EDIT2,in.mp);
		DDV_MinMaxFloat(pdx,in.mp,0.00f,(float)(in.mc/2.00f));
		
		DDX_Text(pdx,IDC_EDIT3,in.l);
		DDV_MinMaxFloat(pdx,in.l,0.00f,10.00f);

		DDX_Text(pdx,IDC_EDIT4,in.ang);
		DDV_MinMaxFloat(pdx,in.ang,-45.00f,45.00f);

		DDX_Text(pdx,IDC_EDIT5,in.vp);
		DDV_MinMaxFloat(pdx,in.vp,-10.00f,10.00f);

		DDX_Text(pdx,IDC_EDIT6,in.step);
		DDV_MinMaxFloat(pdx,in.step,0.00f,5.00f);

		DDX_Text(pdx,IDC_EDIT7,in.max_acl);
		DDV_MinMaxFloat(pdx,in.max_acl,0.00f,30.00f);

		DDX_Text(pdx,IDC_EDIT8,in.max_vel);
		DDV_MinMaxFloat(pdx,in.max_vel,0.00f,40.00f);
	
	
	
	}	
	
	
	void OnOK()
	{
		if(UpdateData(TRUE))
		{
			CDialog::EndDialog(IDOK);
			in.nang_a=4.5;
		}
		inputgiven=TRUE; //input has been given
	}

};


/*Class To Create the main window along with menu*/
class myframe:public CFrameWnd
{
private:
	CMenu mymenu;  //menu class

public: 
	myframe()
	{
		Create(0,"Inverted Pendulum");
	}

	int OnCreate(LPCREATESTRUCT l) //on Creating the window
	{
		CFrameWnd::OnCreate(l);
		mymenu.LoadMenu(IDR_MENU1);  //show the menu
		SetMenu(&mymenu);

		init_rules(); //initialise the rules for controlling the vehicle 
		init_val();    //assign the range of fuzzy sets
		return 0;
	}


	// Display info about the author in the dialog box
	void about()
	{
	aboutbox d(IDD_DIALOG2); //box created and d is an object of the class aboutbox
	d.DoModal();
	}

	// Display help in the dialog box
	void help()
	{
	helpbox d(IDD_DIALOG3); //box created and d is an object of the class helpbox
	d.DoModal();
	}
	
	// Take input in the dialog box
	void input()
	{
		inputbox d(IDD_DIALOG1); //box created and d is an object of the class inputbox
		d.DoModal();
	}

	//calculatin the new angular pos. and velocity of pendulum
	void calculate(float sv,float sa,double *velc)
	{
		
		double area[8],sum=0.0,wt_sum=0.0,temp;
		float ang_int[4],vel_int[4],trap[8][3];
		int midpoint,endpoint,count=0,ind1[4],ind2[4],i,j,mat[8],ang_count=0,vel_count=0,t_count=0;
		

		CClientDC toscr(this);
		
		//Calculate intersections of present angle with fuzzy sets of angles
		
		ang_count=0;
		if(sa>=4)
		{
			ang_int[ang_count]=1;
			ind1[ang_count]=4;
		
			ang_count++;
		}
		else if(sa<=-4)
		{
			ang_int[ang_count]=1;
			ind1[ang_count]=0;
			ang_count++;
		}

		else
		{

		for(i=0;i<5;i++)
		{
			midpoint=val[i][1]; //assign the midpoint of fuzzy set
			for(j=0;j<2;j++)
			{
				if(sa<=val[i][j+1] && sa>=val[i][j]) //check in which fuzzy set the angle lies
				{
					if(val[i][j]==midpoint)
						endpoint=val[i][j+1];
					else
						endpoint=val[i][j];
					ang_int[ang_count]=(float)((endpoint-sa)/(endpoint-midpoint)); //store the intersections
					ind1[ang_count]=i; //store the position of intersection
					ang_count++;
				}
			}
		}
		}


		//Calculate intersections of present velocity with fuzzy sets of velocity
		vel_count=0;
		if(sv>=4)
		{
			vel_int[vel_count]=1;
			ind2[vel_count]=4;
			vel_count++;
		}
		else if(sv<=-4)
		{
			vel_int[vel_count]=1;
			ind2[vel_count]=0;
			vel_count++;
		}

		else
		{


		for(i=0;i<5;i++)
		{
			midpoint=val[i][1]; //assign the midpoint of fuzzy set
			for(j=0;j<2;j++)
			{
				if(sv<=val[i][j+1] && sv>=val[i][j])
				{
					if(val[i][j]==midpoint)
						endpoint=val[i][j+1];
					else
						endpoint=val[i][j];
					vel_int[vel_count]=(float)((endpoint-sv)/(endpoint-midpoint));
					ind2[vel_count]=i;
					vel_count++;
				} //end if 
			} //end inner for
		}//end outer for
		} //end else

		count=0;
				
		for(i=0;i<vel_count;i++)
		{
			for(j=0;j<ang_count;j++)
			{
				mat[count]=rule[ind2[i]][ind1[j]];   //get the corresponding set in rule base
				count++;
			}
		}

		t_count=0;
		for(i=0;i<vel_count;i++)
		{
			for(j=0;j<ang_count;j++)
			{
				trap[t_count][0]=(float)val[mat[t_count]][0];
				
				trap[t_count][1]=(float)val[mat[t_count]][2];
				
				trap[t_count][2]=ang_int[j]>vel_int[i]?vel_int[i]:ang_int[j];
				t_count++;
			}
		}

		sum=0;
		wt_sum=0;
		/*sprintf(buf,"t_count=%d",t_count);
		MessageBox(buf,"message");*/
		for(i=0;i<t_count;i++)
		{
			area[i]=0.5*trap[i][2] * (2-trap[i][2])*(trap[i][1]-trap[i][0]);
			sum+=area[i];
			wt_sum+=area[i]*0.5*(trap[i][1]+trap[i][0]);
		}
		
		sum=wt_sum/sum;
		
		if(in.step!=0)
		{
			temp=sum/in.step;
			temp=int(temp);
			temp=temp*in.step;
			sum=temp;
		}

		*velc=sum;
		
	}

	void delay(int time)
	{
		for(int j=0;j<time*100000;j++); //time=900 means a delay of 1 second
	}

	//Error handling: no input
	void error_noinput()
	{
		MessageBox("Please Give the Input First!","Input Unavailable",MB_OK|MB_ICONEXCLAMATION);
	}
	
	//draw the graph
	void graph_out()
	{
	
		if(!inputgiven) //no input available
		{
			error_noinput();
			return;
		}

		
		int oldangx,oldangy,newangx,newangy,oldvelx,oldvely,newvely,newvelx;
		int oldvelcx,oldvelcy,newvelcx,newvelcy;
		float ang_temp=in.ang, vel_temp=in.vp;
		float ang_temp2;
		double vel_cart=0,a;		
		float x1=ang_temp,x2=vel_temp,l=in.l,mc=in.mc,m=in.mp;
		const double g=9.8;
		char buf[80];
				
		
		int myiterator=0;
		gr=1;
		md=0;
		int xorigin,yorigin;
		float maxvelc=0,maxangp=0,maxvelp=0;
		CClientDC p(this);
		CRect r,mes;
		CPen mypen[3];
		mypen[0].CreatePen(PS_SOLID,3,RGB(255,10,10));
		mypen[1].CreatePen(PS_SOLID,3,RGB(10,155,0));
		mypen[2].CreatePen(PS_SOLID,3,RGB(10,10,255));
		GetClientRect(&r);
		xorigin=r.left+100;
		yorigin=(r.bottom-r.top)/2;
		
		//draw axes with origin at (xorigin,yorigin)
		p.Rectangle(0,0,1000,1000);
		p.SetViewportOrg(xorigin,yorigin);
		p.SetMapMode(MM_LOENGLISH);
		p.MoveTo(0,0);
		p.LineTo(0,410);
		
		p.MoveTo(0,0);
		p.LineTo(0,-410);	
		
		p.MoveTo(0,210);
		p.LineTo(700,210);
		
		p.MoveTo(0,-210);
		p.LineTo(700,-210);
		
		p.MoveTo(0,0);
		p.LineTo(700,0);

		mes.bottom=20;
		mes.top=350;
		mes.left=-170;
		mes.right=-10;
		p.DrawText("Angle \nof \nthe \npendulum\n(degrees)",-1,mes,DT_WORDBREAK|DT_CENTER);
		
		mes.bottom=-220;
		mes.top=70;
		mes.left=-170;
		mes.right=-10;
		p.DrawText("Velocity\nof \nthe \nCart\n(m/s)",-1,mes,DT_WORDBREAK|DT_CENTER);
		

		mes.bottom=-450;
		mes.top=-220;
		mes.left=-170;
		mes.right=-10;
		p.DrawText("Velocity\nof \nthe \nPendulum\n(m/s)",-1,mes,DT_WORDBREAK|DT_CENTER);
		
		//while the controller doesn't fail or 40 cycles haven't been completed
		
		while(ang_temp>-45 && ang_temp<45 && myiterator<100) 
		{
			oldangx=myiterator;
			oldangy=(int)ang_temp;
			oldvelx=myiterator;
			oldvely=(int)vel_temp;
			oldvelcx=myiterator;
			oldvelcy=vel_cart;
			ang_temp2=ang_temp/in.nang_a;
			calculate(vel_temp,ang_temp2,&vel_cart);
			vel_cart*=mc;

			if(fabs(vel_cart)>in.max_vel)
				{
					if(vel_cart<=0)
						vel_cart=-in.max_vel;
					else
						vel_cart=in.max_vel;
				}	
			if(fabs(maxvelc)<fabs(vel_cart))
				maxvelc=fabs(vel_cart);
			if(fabs(maxvelp)<fabs(vel_temp))
				maxvelp=fabs(vel_temp);
			if(fabs(maxangp)<fabs(ang_temp))
				maxangp=fabs(ang_temp);
			
			a=g*sin(rad(x1));
			a-=m*l*pow(x2,2)*cos(rad(x1))*sin(rad(x1))/(mc+m);
			a/=l*(4/3-m*pow(cos(rad(x1)),2)/(mc+m));
			a-=(vel_cart*cos(rad(x1))/(mc+m))/(l*(4/3-m*pow(cos(rad(x1)),2)/(mc+m)));
			
		
			a=fabs(in.max_acl)<fabs(a)?(a<0?(-in.max_acl):in.max_acl):a;
			
			
			ang_temp=ang_temp+vel_temp+0.5*a;
			vel_temp=vel_temp+a;
						
			myiterator++;
			x1=ang_temp;
			x2=vel_temp;
			
			newangx=myiterator;
			newangy=(int)ang_temp;
			newvelx=myiterator;
			newvely=(int)vel_temp;
			newvelcx=myiterator;
			newvelcy=int(vel_cart);
			
			
			p.SelectObject(&mypen[1]);
			p.MoveTo(oldvelcx*5,oldvelcy*25/mc);
			p.LineTo(newvelcx*5,newvelcy*25/mc);
			
			p.SelectObject(&mypen[0]);
			p.MoveTo(oldangx*5,oldangy*10+210);
			p.LineTo(newangx*5,newangy*10+210);
			
			p.SelectObject(&mypen[2]);
			p.MoveTo(oldvelx*5,oldvely*10-210);
			p.LineTo(newvelx*5,newvely*10-210);
			
	        delay(60);
			
			/*sprintf(buf,"x1=%lf",x1);
			MessageBox(buf,"message");*/
					

		}
		for(int i=0;i<2;i++)
		{
			p.MoveTo(0,(25*(maxvelc/2)*(i+1))/mc);
			p.LineTo(-10,25*(maxvelc/2)*(i+1)/mc);
			sprintf(buf,"%.0f",(maxvelc/2)*(i+1));
			p.TextOut(-30,25*(maxvelc/2)*(i+1)/mc+10,buf);

			p.MoveTo(0,10*(maxangp/2)*(i+1)+210);
			p.LineTo(-10,10*(maxangp/2)*(i+1)+210);
			sprintf(buf,"%.0f",(maxangp/2)*(i+1));
			p.TextOut(-30,10*(maxangp/2)*(i+1)+210+10,buf);

			
			p.MoveTo(0,10*(maxvelp/2)*(i+1)-210);
			p.LineTo(-10,10*(maxvelp/2)*(i+1)-210);
			sprintf(buf,"%.0f",(maxvelp/2)*(i+1));
			p.TextOut(-30,10*(maxvelp/2)*(i+1)-210+10,buf);


			if(i==0)
			{
			p.MoveTo(0,-10*(maxangp/2)*(i+1)+210);
			p.LineTo(-10,-10*(maxangp/2)*(i+1)+210);
			sprintf(buf,"%.0f",-(maxangp/2)*(i+1));
			p.TextOut(-30,-10*(maxangp/2)*(i+1)+210+10,buf);
			}
		
			p.MoveTo(0,(-25*(maxvelc/2)*(i+1))/mc);
			p.LineTo(-10,-25*(maxvelc/2)*(i+1)/mc);
			sprintf(buf,"%.0f",-(maxvelc/2)*(i+1));
			p.TextOut(-30,-25*(maxvelc/2)*(i+1)/mc+10,buf);



		}

		for (i=0;i<=10;i++)
		{
			sprintf(buf,"%d",i*10);
			p.TextOut(i*50+5,-300,buf);
		}
		p.TextOut(200,-350,"Time(seconds) --->");
		
	
		if(myiterator<100)
		{
			sprintf(buf,"Fuzzy controller failed after %d cycles (as |angle|>=45)",myiterator);
			MessageBox(buf,"Controller failed !",MB_OK|MB_ICONSTOP);
		}



	}

	void model_visualization()
	{
		if(!inputgiven)
		{
			error_noinput();
			return;
		}
				
		int xorigin,yorigin;
		CRect r;
		GetClientRect(&r);
		xorigin=(r.right+r.left)/2;
		yorigin=r.bottom;
		
		CClientDC p(this);
		
		p.Rectangle(0,0,1000,1000);
		
		p.SetViewportOrg(xorigin,yorigin);
		p.SetMapMode(MM_LOENGLISH);
		
		int indec=0;
		int myiterator=0;
		gr=0;
		md=1;
		float ang_temp=in.ang, vel_temp=in.vp;
		float ang_temp2;
		double vel_cart=0,a;		
		float x1=ang_temp,x2=vel_temp,l=in.l,mc=in.mc,m=in.mp;
		const double g=9.8;
		double distance=0;
		double cart_acl=0;
		float prev_vel=vel_cart,prev_angle=ang_temp;
		float prev_distance=0;
		
		drawimage(ang_temp,distance,prev_angle,prev_distance,&p);
		while(ang_temp>-45 && ang_temp<45 && myiterator<100) 
		{
			if(prev_distance!=distance)
			{
			if(prev_distance<distance)
			{
				indec=0;
				while((prev_distance)<distance)
				{
					drawimage(ang_temp,prev_distance+indec,prev_angle,prev_distance,&p);
					prev_distance+=indec;
					indec++;
				}
			}
			else
			{
				indec=0;
				while((prev_distance)>distance)
				{
					drawimage(ang_temp,prev_distance-indec,prev_angle,prev_distance,&p);
					prev_distance-=indec;
					indec++;
				}
			}
			}
				


			//prev_distance=distance;
			prev_angle=ang_temp;
			ang_temp2=ang_temp/in.nang_a;
			prev_vel=vel_cart;
			calculate(vel_temp,ang_temp2,&vel_cart);
			vel_cart*=mc;
			if(fabs(vel_cart)>in.max_vel)
				{
					if(vel_cart<=0)
						vel_cart=-in.max_vel;
					else
						vel_cart=in.max_vel;
				}	
			
			a=g*sin(rad(x1));
			a-=m*l*pow(x2,2)*cos(rad(x1))*sin(rad(x1))/(mc+m);
			a/=l*(4/3-m*pow(cos(rad(x1)),2)/(mc+m));
			a-=(vel_cart*cos(rad(x1))/(mc+m))/(l*(4/3-m*pow(cos(rad(x1)),2)/(mc+m)));
			
			/*sprintf(buf,"cart acceleration should be=%lf",a);
			MessageBox(buf,"message");*/
			
			a=fabs(in.max_acl)<fabs(a)?(a<0?(-in.max_acl):in.max_acl):a;
			
			//sprintf(buf,"cart acceleration will be=%lf",a);
			//MessageBox(buf,"message");
			
			ang_temp=ang_temp+vel_temp+0.5*a;
			vel_temp=vel_temp+a;
			cart_acl=vel_cart-prev_vel;
			distance=distance+vel_cart+0.5*cart_acl;
			x1=ang_temp;
			x2=vel_temp;
			myiterator++;
	
		}

		char buf[80];
		if(myiterator<100)
		{
			sprintf(buf,"Fuzzy controller failed after %d cycles (as |angle|>=45)",myiterator);
			MessageBox(buf,"Controller failed !",MB_OK|MB_ICONSTOP);
		}
	}
	void drawimage(double angle,double cart_pos,double last_angle,double last_pos,CDC *p)
	{
		
		char buf[80];
		int x1,x2;
		int xpos,ypos;
		float xc,yc;
		float theta;
		float xadd;
		float yadd;
		float theta2;
		float xadd2;
		float yadd2;
		
		x1=-120;x2=120;
		last_pos=int(last_pos)%300;
		cart_pos=int(cart_pos)%300;
		
		theta=-10*cart_pos;
		xc=(x1+20+cart_pos*2+x1+70+cart_pos*2)/2;
		yc=85;
		xadd=xc+15*cos(rad(theta));
		yadd=yc+15*sin(rad(theta));
		
		theta2=-10*last_pos;
		xc=(x1+20+last_pos*2+x1+70+last_pos*2)/2;
		yc=85;
		xadd2=xc+15*cos(rad(theta2));
		yadd2=yc+15*sin(rad(theta2));
		
		CBrush mybrush[5];
		CPen mypen;
		CPen  mypen2;
		mybrush[1].CreateSolidBrush(RGB(0,0,0));	
		mybrush[0].CreateSolidBrush(RGB(255,255,255));
		p->SelectObject(&mybrush[0]);
		mypen.CreatePen(PS_SOLID,10,RGB(255,255,255));
		p->SelectObject(&mypen);
		p->Rectangle(-1000,1000,1000,202);
		mypen.DeleteObject();	
	
		if(last_pos!=cart_pos)
		{
			p->Ellipse(int(x1+20+last_pos*2),110,int(x1+70+last_pos*2),60);
			p->Ellipse(int(x2-70+last_pos*2),110,int(x2-20+last_pos*2),60);
			circle(int(xadd2),yadd2,5,p);
			circle(int(xadd2+152),yadd2,5,p);

			p->Rectangle(int(x1+last_pos*2),200,int(x2+last_pos*2),100);
		}
		
		
		mypen.CreatePen(PS_SOLID,1,RGB(0,0,0));
		p->SelectObject(&mypen);
		p->SelectObject(&mybrush[1]);
		p->Ellipse(int(x1+20+cart_pos*2),110,int(x1+70+cart_pos*2),60);
		p->Ellipse(int(x2-70+cart_pos*2),110,int(x2-20+cart_pos*2),60);
			
		mybrush[2].CreateSolidBrush(RGB(255,255,0));
		p->SelectObject(&mybrush[2]);
		circle(int(xadd),yadd,5,p);
		circle(int(xadd+152),yadd,5,p);	

		mybrush[3].CreateSolidBrush(RGB(170,170,0));
		p->SelectObject(&mybrush[3]);
		p->Rectangle(int(x1+cart_pos*2),200,int(x2+cart_pos*2),100);
		mypen.DeleteObject();
		mybrush[3].DeleteObject();
		mybrush[3].CreateSolidBrush(RGB(0,0,255));
		p->SelectObject(&mybrush[3]);
		
		mypen.CreatePen(PS_SOLID,10,RGB(0,0,255));
		
		p->SelectObject(&mypen);
		p->MoveTo(int((x1+x2+2*cart_pos*2)/2-10),200);
		xpos=int(cos(rad(90-angle*1.8))*300)+cart_pos*2;
		ypos=int(sin(rad(90-angle*1.8))*300)+200;
		p->LineTo(int(xpos),ypos);
		mypen2.CreatePen(PS_SOLID,2,RGB(0,0,0));
		p->SelectObject(&mypen2);
		circle(xpos,ypos,20,p);
		mybrush[4].CreateSolidBrush(RGB(255,255,0));
		p->SelectObject(&mybrush[4]);
		circle(xpos+5,ypos,5,p);		
		mypen.DeleteObject();
		
		
		CBrush myb;
		CPen myp;
		myp.CreatePen(PS_SOLID,1,RGB(0,0,0));
		p->SelectObject(&myp);
		myb.CreateSolidBrush(RGB(0,0,0));
		p->SelectObject(&myb);
		p->Rectangle(-1000,60,1000,0);
		delay(80);
		
	}

			
		
	void circle(int x,int y,int rad,CDC *p)
	{
		p->Ellipse(x-rad,y+rad,x+rad,y-rad);
	}


	double rad(double angle)
	{
		angle=angle*PI/180;
		return(angle);
	}
	
DECLARE_MESSAGE_MAP() //message map will handle the required events
};

/*Macros to map the menu command with the respective function*/

BEGIN_MESSAGE_MAP(myframe,CFrameWnd)
ON_WM_CREATE()
ON_WM_KEYDOWN()
ON_COMMAND(105,about)
ON_COMMAND(104,help)
ON_COMMAND(101,input)
ON_COMMAND(102,graph_out)
ON_COMMAND(103,model_visualization)
END_MESSAGE_MAP()

/* Application class*/
class myapp:public CWinApp
{
int InitInstance()
	{
		myframe *p;
		p=new myframe;
		p->ShowWindow(3);  //show the application maximized at the startup
		m_pMainWnd=p;

		return 1;
	}
};

myapp a; //execute the program




