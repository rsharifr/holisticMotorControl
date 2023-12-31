/***************************************************
 * Automatically generated by Maple.
 * Created On: Mon Jul 31 12:00:08 2023.
***************************************************/
#ifdef WMI_WINNT
#define EXP __declspec(dllexport)
#else
#ifdef X86_64_WINDOWS
#define EXP __declspec(dllexport)
#else
#define EXP
#endif
#endif
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifdef FROM_MAPLE
#include <mplshlib.h>
static MKernelVector kv;
EXP ALGEB M_DECL SetKernelVector(MKernelVector kv_in, ALGEB args) { ((void)(args)); kv=kv_in; return(kv->toMapleNULL()); }
#include <string.h>
#else
#include <string.h>
#include <stdarg.h>

#define MAXSTRINGLENGTH 1000
#define MAXNUMPART 20


void toMapleError(char *msg, ...)
{
	char fcpy[MAXSTRINGLENGTH+1],*offs[MAXNUMPART+1],
		*msg_pars[MAXNUMPART+1],res_str[MAXSTRINGLENGTH+1];
	int argref[MAXNUMPART],maxargref,npart,v,i,j,res_len;
	va_list argptr;

	if(strlen(msg)>MAXSTRINGLENGTH)
		strcpy(fcpy,"message string is too long");
	else
		strcpy(fcpy,msg);

	maxargref=0;
	npart=0;
	i=0;
	offs[npart]=&fcpy[i];
	for(j=0;fcpy[j]!='\0';j++)
		if(fcpy[j]=='%' && fcpy[j+1]!='%') {
			fcpy[j++]='\0';
			if(fcpy[j]=='-') j++;
			if(fcpy[j]=='\0') break;
			v=0;
			while(fcpy[j]>='0' && fcpy[j]<='9') v=10*v+(fcpy[j++]-'0');
			argref[npart]=v;
			if(v>maxargref) maxargref=v;
			i=j;
			npart++;
			offs[npart]=&fcpy[i];
		}

	va_start(argptr,msg);
	for(i=0;i<maxargref;i++) msg_pars[i] = va_arg(argptr, char*);
	va_end(argptr);

	strcpy(res_str,offs[0]);
	res_len=strlen(res_str);
	for(i=0;i<npart;i++) {
		res_len+=strlen(msg_pars[argref[i]-1]);
		if(res_len>MAXSTRINGLENGTH) {
			strcpy(res_str,"message string is too long");
			break;
		}
		else
			strcat(res_str,msg_pars[argref[i]-1]);

		res_len+=strlen(offs[i+1]);
		if(res_len>MAXSTRINGLENGTH) {
			strcpy(res_str,"message string is too long");
			break;
		}
		else
			strcat(res_str,offs[i+1]);
	}
	for(i=0;i<maxargref;i++) free(msg_pars[i]);
	fprintf(stderr,"Error: %s\n",res_str);
}

char* toMapleInteger(int i)
{
	char *str=(char *)malloc(21);
	sprintf(str,"%li",(long)i);
	return(str);
}

char* toMapleFloat(double f)
{
	char *str=(char *)malloc(20);
	sprintf(str,"%f",f);
	return(str);
}

char* toMapleString(const char *s)
{
	char *str=(char *)malloc(strlen(s)+1);
	strcpy(str,s);
	return(str);
}

char* toMapleBoolean(int b)
{
	char *str=(char *)malloc(6);
	switch(b) {
	case 0:
		strcpy(str,"false");
	default:
		strcpy(str,"true");
	return(str);
	}
}

void toMapleUserinfo(int level, char *name, char *msg)
{
	printf("Info: %s:%s\n",name,msg);
}

typedef struct MKernelVectorDesc {
    void (*error)(char *msg, /* char *par1, *par2, */ ...);
	char* (*toMapleInteger)(int i);
	char* (*toMapleFloat)(double f);
	char* (*toMapleString)(const char *s);
	char* (*toMapleBoolean)(int b);
	void (*userinfo)(int level, char *name, char *msg);
} MKernelVectorDesc, *MKernelVector;

MKernelVectorDesc mykv = {
	&toMapleError,
	&toMapleInteger,
	&toMapleFloat,
	&toMapleString,
	&toMapleBoolean,
	&toMapleUserinfo
};

MKernelVector kv= &mykv;
#ifdef WMI_WINNT
#define M_DECL __stdcall
#else
#define M_DECL
#endif
#endif

/***************************************************
* Variable Definition for System:

* State variable(s):
*    x[ 0] = `Main.PlanarArm.MuscleArm1.BiExtensor.activationDynamics1.a`(t)
*    x[ 1] = `Main.PlanarArm.MuscleArm1.BiFlexor.activationDynamics1.a`(t)
*    x[ 2] = `Main.PlanarArm.MuscleArm1.ElbowExtensor.activationDynamics1.a`(t)
*    x[ 3] = `Main.PlanarArm.MuscleArm1.ElbowFlexor.activationDynamics1.a`(t)
*    x[ 4] = `Main.PlanarArm.MuscleArm1.ShoulderExtensor.activationDynamics1.a`(t)
*    x[ 5] = `Main.PlanarArm.MuscleArm1.ShoulderFlexor.activationDynamics1.a`(t)
*    x[ 6] = `Main.PlanarArm.MuscleArm1.zElbow.theta`(t)
*    x[ 7] = diff(`Main.PlanarArm.MuscleArm1.zElbow.theta`(t),t)
*    x[ 8] = `Main.PlanarArm.MuscleArm1.zShoulder.theta`(t)
*    x[ 9] = diff(`Main.PlanarArm.MuscleArm1.zShoulder.theta`(t),t)
*
* Output variable(s):
*    y[ 0] = `Main.PlanarArm.MuscleArm1.zElbow.theta`(t)
*    y[ 1] = diff(`Main.PlanarArm.MuscleArm1.zElbow.theta`(t),t)
*    y[ 2] = `Main.PlanarArm.MuscleArm1.zShoulder.theta`(t)
*    y[ 3] = diff(`Main.PlanarArm.MuscleArm1.zShoulder.theta`(t),t)
*    y[ 4] = `Main.PlanarArm.axyz[1]`(t)
*    y[ 5] = `Main.PlanarArm.axyz[2]`(t)
*    y[ 6] = `Main.PlanarArm.axyz[3]`(t)
*    y[ 7] = `Main.PlanarArm.vxyz[1]`(t)
*    y[ 8] = `Main.PlanarArm.vxyz[2]`(t)
*    y[ 9] = `Main.PlanarArm.vxyz[3]`(t)
*    y[10] = `Main.PlanarArm.xyz[1]`(t)
*    y[11] = `Main.PlanarArm.xyz[2]`(t)
*    y[12] = `Main.PlanarArm.xyz[3]`(t)
*    y[13] = `Main.PlanarArm.FBiExt`(t)
*    y[14] = `Main.PlanarArm.FBiFlex`(t)
*    y[15] = `Main.PlanarArm.FElExt`(t)
*    y[16] = `Main.PlanarArm.FElFlex`(t)
*    y[17] = `Main.PlanarArm.FShExt`(t)
*    y[18] = `Main.PlanarArm.FShFlex`(t)
*
* Input variable(s):
*    u[ 0] = `Main.PlanarArm.Fx`(t)
*    u[ 1] = `Main.PlanarArm.Fz`(t)
*    u[ 2] = `Main.PlanarArm.uBiExt`(t)
*    u[ 3] = `Main.PlanarArm.uBiFlex`(t)
*    u[ 4] = `Main.PlanarArm.uElExt`(t)
*    u[ 5] = `Main.PlanarArm.uElFlex`(t)
*    u[ 6] = `Main.PlanarArm.uShExt`(t)
*    u[ 7] = `Main.PlanarArm.uShFlex`(t)
*
* Parameter(s):
*    p[ 0] = `Main.PlanarArm.KdEl` (default = 1.)
*    p[ 1] = `Main.PlanarArm.KdSh` (default = 1.)
*
************************************************/

/* Set up visible initial condition mask */
int icvis[10] = {
	1,1,1,1,1,1,1,1,1,1
};

#define INCONTOL 1e200

/* Wordsize integer definition */
#ifndef M_INT
# if defined _MINGW64 || defined X86_64_WINDOWS
#  define M_INT long long
# elif defined __x86_64__ || defined __ppc64__
#  define M_INT long
# elif defined _M_X64
#  define M_INT long long
# else
#  define M_INT long
# endif
#endif

/* Fixed parameters */
#define NDIFF 10
#define NDFA 10
#define NSDIFF 10
#define NEQ 59
#define NPAR 2
#define NINP 8
#define NDISC 0
#define NIX1 49
#define NOUT 19
#define NCON 0
#define NEVT 0
#ifdef EVTHYST
#define NZC 2*NEVT
#else
#define NZC NEVT
#endif

typedef struct {
	double h;		/* Integration step size */
	double *w;		/* Float workspace */
	int *iw;		/* Integer workspace */
	int err;			/* Error flag */
	char *buf;		/* Error message */
	double py[NOUT];
} SolverStruct;

static void SolverError(SolverStruct *S, int term, char *errmsg)
{
#ifdef FROM_MAPLE
	int i;
#endif
	if(term)
		sprintf(S->buf,"Simulation terminated at t=%20.16e: %s\n",S->w[0],errmsg);
	else
		sprintf(S->buf,"Error at t=%20.16e: %s\n",S->w[0],errmsg);
#ifdef FROM_MAPLE
	for(i=0;S->buf[i]!='\0';i++);
	S->buf[i-1]='\0';
	if(S->err==-1) kv->error(S->buf);
#endif
	S->err=1;
}

static double dsn_zero=0.0;
static unsigned char dsn_undefC[8] = { 0, 0, 0, 0, 0, 0, 0xF8, 0x7F };
static double *dsn_undef = (double *)&dsn_undefC;
static unsigned char dsn_posinfC[8] = { 0, 0, 0, 0, 0, 0, 0xF0, 0x7F };
static double *dsn_posinf = (double *)&dsn_posinfC;
static unsigned char dsn_neginfC[8] = { 0, 0, 0, 0, 0, 0, 0xF0, 0xFF };
static double *dsn_neginf = (double *)&dsn_neginfC;
#define trunc(v) ( (v>0.0) ? floor(v) : ceil(v) )
#define IS_UNDEF(a) (a-a!=0. || (a!=0. && a-2.*a==0.))
#define UNUSED(a) ((void)(a))
double avoidcompilerwarn() { return(dsn_zero+*dsn_undef+*dsn_posinf+*dsn_neginf); }
#ifndef M_INT
# if defined _MINGW64 || defined X86_64_WINDOWS
#  define M_INT long long
# elif defined __x86_64__ || defined __ppc64__
#  define M_INT long
# elif defined _M_X64
#  define M_INT long long
# else
#  define M_INT long
# endif
#endif


#ifdef MSVC
#pragma optimize( "gty", on)
#endif
#ifndef INCONTOL
#define INCONTOL 1e-14
#endif

static void LSQDecompSolve(M_INT n, double *A, M_INT Ainc, M_INT *ip, double *b)
{
	M_INT i,j,k,r,c,lsq,cp;
	double s,t;

	for(i=0;i<n;i++) {
		if(b[i]-b[i]!=0.0 || (b[i]!=0. && b[i]-2.*b[i]==0.)) { ip[n-1]=0; A[0]=1.; for(j=0;j<n;j++) b[j]=*dsn_undef; return; }
		t = fabs(A[i*Ainc]);
		for(j=1;j<n;j++) {
			s = fabs(A[i*Ainc+j]);
			if(s>t) t = s;
		}
		if( t==0.0 )
			A[i*Ainc+n] = 1.0;
		else {
			t = 1.0/t;
			for(j=0;j<n;j++) A[i*Ainc+j] *= t;
			b[i] *= t;
			A[i*Ainc+n] = t;
		}
	}

	for(j=0;j<n;j++) {
		t = fabs(A[j]);
		for(i=1;i<n;i++) {
			s = fabs(A[i*Ainc+j]);
			if(s>t) t = s;
		}
		if( t==0.0 )
			A[n*Ainc+j] = 1.0;
		else {
			t = 1.0/t;
			for(i=0;i<n;i++) A[i*Ainc+j] *= t;
			A[n*Ainc+j] = t;
		}
	}

	ip[n-1] = 0;
	lsq = 0;
	cp = 0;
	for(k=0;k<n;k++) {
		r = 0; c = 0; t = 0.0;
		if( cp<n ) {
			for(cp=cp+1;cp<n;cp++) {
				for(i=k;i<n;i++) {
					s = fabs(A[i*Ainc+cp]);
					if(s>t) { r = i; c = cp; t = s; }
				}
				if( t>0.5 ) break;
			}
		}
		if( cp>=n ) {
			for(i=k;i<n;i++)
				for(j=k;j<n;j++) {
					s = fabs(A[i*Ainc+j]);
					if(s>t) { r = i; c = j; t = s; }
				}
		}
		if( t<1e-10 ) {
			if( k==0 ) { ip[n-1]=0; A[0]=2.; for(j=0;j<n;j++) b[j]=*dsn_undef; return; }
			if( lsq ) { ip[n-1]=0; A[0]=3.; for(j=0;j<n;j++) b[j]=*dsn_undef; return; }
			lsq = 1;
			t = 1e-16;
			for(i=0;i<k;i++) {
				s = fabs(b[i]);
				if(s>t) t = s;
			}
			for(i=k;i<n;i++) {
				if( fabs(b[i])/t>INCONTOL*n ) { ip[n-1]=0; A[0]=4.; for(j=0;j<n;j++) b[j]=*dsn_undef; return; }
				b[i] = 0.0;
			}

			for(r=k;r<n;r++) {
				for(i=k;i<n;i++) A[i*Ainc+n+r] = 0.0;
				A[r*Ainc+n+r] = 1.0;
				A[(k-1)*Ainc+n+r] = -A[(k-1)*Ainc+r]/A[(k-1)*Ainc+k-1];
				for(i=k-1;i>=0;i--) {
					A[i*Ainc+n+r] = A[i*Ainc+r];
					for(j=i+1;j<k;j++)
						A[i*Ainc+n+r] += A[i*Ainc+j]*A[j*Ainc+n+r];
					A[i*Ainc+n+r] = -A[i*Ainc+n+r]/A[i*(Ainc+1)];
				}
			}
			for(r=0;r<k;r++) {
				s = -1.0/A[r*(Ainc+1)];
				for(i=k;i<n;i++)
					if( A[r*Ainc+n+i]!=0.0 ) {
						t = s*A[r*Ainc+n+i];
						A[i*Ainc+r] += t;
						for(j=r+1;j<n;j++)
							A[j*Ainc+n+i] += t*A[r*Ainc+j];
						b[i] += t*b[r];
					}
			}
			for(i=k;i<n;i++)
				for(j=k;j<n;j++)
					A[i*Ainc+j] = A[j*Ainc+n+i];
			r = 0; c = 0; t = 0.0;
			for(i=k;i<n;i++)
				for(j=k;j<n;j++) {
					s = fabs(A[i*Ainc+j]);
					if(s>t) { r = i; c = j; t = s; }
				}
			if( t<1e-10 ) { ip[n-1]=0; A[0]=3.; for(j=0;j<n;j++) b[j]=*dsn_undef; return; }
		}
		if( !lsq ) ip[n-1]++;
		if( k==n-1 ) break;
		ip[k] = r;
		ip[n+k] = c;
		if( r>k ) {
			for(j=0;j<n;j++) {
				t = A[r*Ainc+j]; A[r*Ainc+j] = A[k*Ainc+j]; A[k*Ainc+j] = t;
			}
			t = b[r]; b[r] = b[k]; b[k] = t;
		}
		if( c>k )
			for(i=0;i<n;i++) {
				t = A[i*Ainc+c]; A[i*Ainc+c] = A[i*Ainc+k]; A[i*Ainc+k] = t;
			}
		s = -1.0/A[k*(Ainc+1)];
		for(i=k+1;i<n;i++)
			if( A[i*Ainc+k]!=0.0 ) {
				t = s*A[i*Ainc+k];
				A[i*Ainc+k] = t;
				for(j=k+1;j<n;j++)
					A[i*Ainc+j] += t*A[k*Ainc+j];
				b[i] += t*b[k];
			}
	}
	for(j=n-1;j>0;j--) {
		b[j] = b[j]/A[j*(Ainc+1)];
		t = -b[j];
		for(i=0;i<j;i++)
			b[i] += t*A[i*Ainc+j];
	}
	b[0] = b[0]/A[0];
	for(j=n-2;j>=0;j--) {
		i = ip[n+j];
		if(i!=j) {
			t = b[i]; b[i] = b[j]; b[j] = t;
		}
	}
	for(j=0;j<n;j++) b[j] *= A[n*Ainc+j];
}
#ifdef MSVC
#pragma optimize( "", on)
#endif

static void fp(int N, double T, double *Y, double *YP)
{
	double M[12], V[2], Z[54];
	int ti1, ti2;
	M_INT P[3];

	UNUSED(N);
	UNUSED(T);
	YP[6] = Y[7];
	YP[8] = Y[9];
	Z[50] = sin(Y[8]);
	Z[51] = sin(Y[6]);
	Z[52] = cos(Y[8]);
	Z[53] = cos(Y[6]);
	Z[0] = Z[50];
	Z[1] = Z[53];
	Z[2] = Z[52];
	Z[3] = Z[51];
	Z[4] = Z[0]*Z[1]+Z[2]*Z[3];
	Z[5] = Z[1]*Z[2]-Z[0]*Z[3];
	Z[6] = -0.03145769966334020184;
	Z[7] = 0.01737446805143240149;
	Z[8] = 0.01869724304842369895;
	Z[9] = 0.31*Z[0];
	Z[6] = Z[4]*Z[7]+Z[5]*Z[8]+Z[6]-Z[9];
	Z[10] = -6.287826248000000487e-08;
	Z[11] = 0.31*Z[2];
	Z[10] = Z[4]*Z[8]+Z[10]+Z[11]-Z[5]*Z[7];
	Z[12] = 115197278836260600.*Z[1]-123967623903226700.*Z[3]-2055381283244332467.;
	Z[13] = -1.076133265955284545;
	Z[14] = Z[1]*Z[13]-Z[3];
	Z[15] = 115197278836260600.*Z[14];
	Z[13] = Z[3]*Z[13]+Z[1];
	Z[16] = 1.508235978050155759e-19;
	Z[7] = Z[7]*(Z[0]*Z[14]+Z[2]*Z[13])*Y[7]+Z[16]*(Z[0]*Z[15]+Z[2]*Z[12])*Y[9];
	Z[8] = Z[16]*(Z[0]*Z[12]-Z[2]*Z[15])*Y[9]+Z[8]*Y[7]*(0.9292529388656144078*(Z[0]*Z[13]-Z[2]*Z[14]));
	Z[12] = Z[6]*Z[6]+Z[10]*Z[10];
	if( Z[12]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[13] = (1.0/(sqrt(Z[12])));
	Z[12] = Z[12]*Z[13];
	Z[13] = (Z[6]*Z[7]+Z[8]*Z[10])*Z[13];
	Z[14] = Z[12]-0.1191861621963549957;
	Z[15] = Z[14]*Z[14]+0.0008878947341260773834;
	Z[16] = (1.0/(sqrt(Z[15])));
	Z[14] = Z[16]*Z[14];
	Y[11] = Z[14]*Z[13];
	if( Y[11]<=0. )
		Y[10] = (2.+6.*Y[0]+0.8721841360538358158*Y[11])/(2.+6.*Y[0]-3.488736544215343263*Y[11]);
	else
		Y[10] = (1.6+4.8*Y[0]+15.69931444896904468*Y[11])/(1.6+4.8*Y[0]+8.721841360538358158*Y[11]);
	Z[15] = -1.+6.977473088430686526*Z[15]*Z[16];
	Y[12] = -798.520000000001*Z[14]*Y[0]*exp(-2.222222222222222222*Z[15]*Z[15])*Y[10];
	Z[14] = 0.04715016227153790061;
	Z[15] = 0.03143025891654109997;
	Z[16] = 0.004258009264589300018;
	Z[9] = Z[5]*Z[16]+Z[15]-Z[4]*Z[14]-Z[9];
	Z[4] = Z[4]*Z[16]+Z[5]*Z[14]+Z[11]+0.009977209043000000111;
	Z[5] = -751595164385510000.*Z[3]-8322629604869613600.*Z[1]-54719115549407157000.;
	Z[11] = 0.0903074148518814035;
	Z[15] = Z[1]*Z[11]-Z[3];
	Z[17] = 8322629604869613600.*Z[15];
	Z[11] = Z[3]*Z[11]+Z[1];
	Z[18] = 5.665296247708788693e-21;
	Z[14] = Z[18]*(Z[2]*Z[5]-Z[0]*Z[17])*Y[9]-Z[14]*(Z[0]*Z[15]+Z[2]*Z[11])*Y[7];
	Z[5] = Z[18]*(Z[0]*Z[5]+Z[2]*Z[17])*Y[9]+Z[16]*(11.07328785393934546*(Z[2]*Z[15]-Z[0]*Z[11]))*Y[7];
	Z[11] = Z[4]*Z[4]+Z[9]*Z[9];
	if( Z[11]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[15] = (1.0/(sqrt(Z[11])));
	Z[11] = Z[11]*Z[15];
	Z[15] = (Z[4]*Z[5]+Z[9]*Z[14])*Z[15];
	Z[16] = Z[11]-0.1876654718923540032;
	if( Z[16]*Z[16]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[17] = Z[16]/sqrt(Z[16]*Z[16]);
	Y[15] = Z[17]*Z[15];
	if( Y[15]<=0. )
		Y[14] = (2.+6.*Y[1]+0.9687894451051341538*Y[15])/(2.+6.*Y[1]-3.875157780420536615*Y[15]);
	else
		Y[14] = (1.6+4.8*Y[1]+17.43821001189241477*Y[15])/(1.6+4.8*Y[1]+9.687894451051341538*Y[15]);
	Z[16] = -1.+7.750315560841073231*fabs(Z[16]);
	Y[16] = -971.6839440783530021*Z[17]*Y[1]*exp(-2.222222222222222222*Z[16]*Z[16])*Y[14];
	Z[16] = 0.001088386836323859039*Z[3]+0.05819953671129420221*Z[1];
	Z[17] = -0.001088386836323859039*Z[1]+0.05819953671129420221*Z[3];
	Z[18] = 0.01939984557043140074*Z[3]-0.0001612100150651070088-0.0003627956121079530131*Z[1];
	Z[19] = -0.01939984557043140074*Z[1]+0.1009578529976279903-0.0003627956121079530131*Z[3];
	Z[20] = Z[18]*Z[18]+Z[19]*Z[19];
	if( Z[20]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[21] = (1.0/(sqrt(Z[20])));
	Z[20] = Z[20]*Z[21];
	Z[21] = 0.3333333333333333333*Y[7]*(Z[16]*Z[18]+Z[17]*Z[19])*Z[21];
	Z[22] = Z[20]-0.007207271244649430134;
	Z[23] = Z[22]*Z[22]+0.000215371568910646809;
	Z[24] = (1.0/(sqrt(Z[23])));
	Z[22] = Z[24]*Z[22];
	Y[19] = Z[22]*Z[21];
	if( Y[19]<=0. )
		Y[18] = (2.+6.*Y[2]+1.149764148384612096*Y[19])/(2.+6.*Y[2]-4.599056593538448384*Y[19]);
	else
		Y[18] = (1.6+4.8*Y[2]+20.69575467092301773*Y[19])/(1.6+4.8*Y[2]+11.49764148384612096*Y[19]);
	Z[23] = -1.+9.198113187076896769*Z[23]*Z[24];
	Y[20] = -1577.017267470980048*Z[22]*Y[2]*exp(-2.222222222222222222*Z[23]*Z[23])*Y[18];
	Z[22] = -0.02486856581214240076;
	Z[23] = Z[1]*Z[22]-0.0008907096092813949785*Z[3];
	Z[22] = Z[3]*Z[22]+0.0008907096092813949785*Z[1];
	Z[24] = 0.001781419218562789957*Z[1]-0.04973713162428480153*Z[3]-0.001196151814771630075;
	Z[1] = 0.001781419218562789957*Z[3]+0.04973713162428480153*Z[1]+0.1001922804;
	Z[3] = Z[1]*Z[1]+Z[24]*Z[24];
	if( Z[3]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[25] = (1.0/(sqrt(Z[3])));
	Z[3] = Z[3]*Z[25];
	Z[25] = 2.*Y[7]*(Z[1]*Z[22]+Z[23]*Z[24])*Z[25];
	Z[26] = Z[3]-0.01811680735798800181;
	Z[27] = Z[26]*Z[26]+8.356635327544370825e-06;
	Z[28] = (1.0/(sqrt(Z[27])));
	Z[26] = Z[28]*Z[26];
	Y[23] = Z[26]*Z[25];
	if( Y[23]<=0. )
		Y[22] = (2.+6.*Y[3]+1.067241872576417184*Y[23])/(2.+6.*Y[3]-4.268967490305668734*Y[23]);
	else
		Y[22] = (1.6+4.8*Y[3]+19.2103537063755093*Y[23])/(1.6+4.8*Y[3]+10.67241872576417184*Y[23]);
	Z[27] = -1.+8.537934980611337468*Z[27]*Z[28];
	Y[24] = -1452.279722084529891*Z[26]*Y[3]*exp(-2.222222222222222222*Z[27]*Z[27])*Y[22];
	Z[26] = -0.01488499887723043396;
	Z[27] = Z[2]*Z[26]-7.399252748545999873e-10*Z[0];
	Z[26] = Z[0]*Z[26]+7.399252748545999873e-10*Z[2];
	Z[28] = -0.04465497329256169882-0.04465499663169130187*Z[0]+2.219775824563799962e-09*Z[2];
	Z[29] = 0.04465499663169130187*Z[2]+2.219775824563799962e-09*Z[0]-2.219790209527749963e-09;
	Z[30] = Z[28]*Z[28]+Z[29]*Z[29];
	if( Z[30]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[31] = (1.0/(sqrt(Z[30])));
	Z[30] = Z[30]*Z[31];
	Z[31] = 3.*Y[9]*(Z[26]*Z[29]+Z[27]*Z[28])*Z[31];
	Z[32] = Z[30]*Z[30]+0.005336516956112195437;
	Z[33] = (1.0/(sqrt(Z[32])));
	Z[34] = Z[33]*Z[30];
	Y[27] = Z[34]*Z[31];
	if( Y[27]<=0. )
		Y[26] = (2.+6.*Y[4]+0.569861042669439647*Y[27])/(2.+6.*Y[4]-2.279444170677758588*Y[27]);
	else
		Y[26] = (1.6+4.8*Y[4]+10.25749876804991365*Y[27])/(1.6+4.8*Y[4]+5.69861042669439647*Y[27]);
	Z[32] = -1.+4.558888341355517176*Z[32]*Z[33];
	Y[28] = -1672.43249129511998*Z[34]*Y[4]*exp(-2.222222222222222222*Z[32]*Z[32])*Y[26];
	Z[32] = -0.01075173575611254275;
	Z[33] = Z[2]*Z[32]+0.003924854783396900058*Z[0];
	Z[32] = Z[0]*Z[32]-0.003924854783396900058*Z[2];
	Z[34] = -0.0274739834837783004*Z[2]+0.03211993553327720254-0.07526215029278779926*Z[0];
	Z[0] = -0.0274739834837783004*Z[0]+0.07179454623553609659+0.07526215029278779926*Z[2];
	Z[2] = Z[0]*Z[0]+Z[34]*Z[34];
	if( Z[2]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[35] = (1.0/(sqrt(Z[2])));
	Z[2] = Z[2]*Z[35];
	Z[35] = 7.*Y[9]*(Z[0]*Z[32]+Z[33]*Z[34])*Z[35];
	Z[36] = Z[2]-0.02916969277103229945;
	Z[37] = Z[36]*Z[36]+0.001959012092791772675;
	Z[38] = (1.0/(sqrt(Z[37])));
	Z[36] = Z[38]*Z[36];
	Y[31] = Z[36]*Z[35];
	if( Y[31]<=0. )
		Y[30] = (2.+6.*Y[5]+1.041121831235231223*Y[31])/(2.+6.*Y[5]-4.164487324940924892*Y[31]);
	else
		Y[30] = (1.6+4.8*Y[5]+18.74019296223416201*Y[31])/(1.6+4.8*Y[5]+10.41121831235231223*Y[31]);
	Z[37] = -1.+8.328974649881849784*Z[37]*Z[38];
	Y[32] = -2524.662523332499958*Z[36]*Y[5]*exp(-2.222222222222222222*Z[37]*Z[37])*Y[30];
	Z[1] = Y[53];
	if( Y[0]<Z[1] )
		Y[13] = 0.0075+0.0225*Y[0];
	else {
		Z[0] = 0.5+1.5*Y[0];
		Z[0] = 1.0/Z[0];
		Y[13] = 0.04*Z[0];
	}
	for(ti1=0;ti1<=1;ti1++)
		for(ti2=0;ti2<=1;ti2++)
			M[ti1*4+ti2] = 0.;
	for(ti1=0;ti1<=1;ti1++)
		V[ti1] = 0.;
	Z[0] = Z[50];
	Z[1] = Z[53];
	Z[2] = Z[52];
	Z[3] = Z[51];
	Z[4] = Z[0]*Z[1]+Z[2]*Z[3];
	Z[5] = Z[1]*Z[2]-Z[0]*Z[3];
	Z[6] = 0.054872;
	Z[6] = Z[6]*(Z[4]*Z[4]+Z[5]*Z[5]);
	M[0] = Z[6]+0.0188;
	Z[7] = 0.2888*Z[4];
	Z[8] = 0.4712*Z[0];
	Z[9] = Z[7]+Z[8];
	Z[10] = 0.2888*Z[5];
	Z[11] = 0.4712*Z[2];
	Z[12] = -(Z[10]+Z[11]);
	Z[13] = 0.19;
	Z[14] = Z[13]*(Z[4]*Z[9]-Z[5]*Z[12]);
	M[1] = Z[14]+0.0188;
	Z[15] = 0.01939984557043140074*Z[1];
	Z[16] = -0.0003627956121079530131*Z[3];
	Z[17] = Z[16]-Z[15]+0.1009578529976279903;
	Z[18] = 0.01939984557043140074*Z[3];
	Z[19] = -0.0003627956121079530131*Z[1];
	Z[20] = Z[18]+Z[19]-0.0001612100150651070088;
	Z[21] = Z[17]*Z[17]+Z[20]*Z[20];
	Z[22] = 0.04973713162428480153*Z[1];
	Z[23] = 0.001781419218562789957*Z[3];
	Z[24] = Z[22]+Z[23]+0.1001922804;
	Z[3] = 0.04973713162428480153*Z[3];
	Z[1] = 0.001781419218562789957*Z[1];
	Z[25] = Z[1]-Z[3]-0.001196151814771630075;
	Z[26] = Z[24]*Z[24]+Z[25]*Z[25];
	Z[27] = 0.01737446805143240149*Z[5];
	Z[28] = 0.31*Z[2];
	Z[29] = 0.01869724304842369895*Z[4];
	Z[30] = Z[29]+Z[28]-Z[27]-6.287826248000000487e-08;
	Z[31] = 0.01737446805143240149*Z[4];
	Z[32] = 0.31*Z[0];
	Z[33] = 0.01869724304842369895*Z[5];
	Z[34] = Z[31]+Z[33]-Z[32]-0.03145769966334020184;
	Z[35] = Z[30]*Z[30]+Z[34]*Z[34];
	Z[36] = 0.04715016227153790061*Z[5];
	Z[37] = 0.004258009264589300018*Z[4];
	Z[28] = Z[28]+Z[36]+Z[37]+0.009977209043000000111;
	Z[38] = 0.04715016227153790061*Z[4];
	Z[39] = 0.004258009264589300018*Z[5];
	Z[32] = Z[39]-Z[32]-Z[38]+0.03143025891654109997;
	Z[40] = Z[28]*Z[28]+Z[32]*Z[32];
	if( Z[35]>1e-15 )
		Z[41] = Z[35];
	else
		Z[41] = 1e-15;
	if( Z[41]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[41] = (1.0/(sqrt(Z[41])));
	if( Z[40]>1e-15 )
		Z[42] = Z[40];
	else
		Z[42] = 1e-15;
	if( Z[42]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[42] = (1.0/(sqrt(Z[42])));
	if( Z[21]>1e-15 )
		Z[43] = Z[21];
	else
		Z[43] = 1e-15;
	if( Z[43]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[43] = (1.0/(sqrt(Z[43])));
	if( Z[26]>1e-15 )
		Z[44] = Z[26];
	else
		Z[44] = 1e-15;
	Z[45] = Y[52];
	Z[46] = Y[51];
	Z[47] = Y[9]+Y[7];
	Z[48] = Y[9]*Y[9];
	Z[47] = 0.2888*Z[47]*Z[47];
	Z[49] = Z[47]*Z[5];
	Z[11] = Z[11]*Z[48];
	Z[47] = Z[47]*Z[4];
	Z[8] = Z[8]*Z[48];
	Z[27] = (Z[34]*(Z[29]-Z[27])-Z[30]*(Z[31]+Z[33]))*Z[41]*Y[12];
	Z[29] = (Z[32]*(Z[36]+Z[37])-Z[28]*(Z[39]-Z[38]))*Z[42]*Y[16];
	if( Z[44]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	Z[31] = Y[24]/sqrt(Z[44]);
	Z[1] = Z[31]*(Z[25]*(Z[22]+Z[23])-Z[24]*(Z[1]-Z[3]));
	Z[3] = (Z[20]*(Z[16]-Z[15])-Z[17]*(Z[18]+Z[19]))*Z[43]*Y[20];
	Z[15] = -0.15;
	Z[13] = Z[13]*(Z[4]*(Z[45]+Z[11]+Z[49])-Z[5]*(Z[46]+Z[8]+Z[47]));
	Z[15] = Z[15]*(Z[4]*Z[45]-Z[5]*Z[46]);
	V[0] = Z[15]-Y[7]*Y[59]-Z[1]-Z[3]-Z[13]-Z[27]-Z[29];
	Z[16] = 0.089528;
	M[4] = Z[16]*(Z[0]*Z[4]+Z[2]*Z[5])+Z[6]+0.0188;
	Z[4] = 0.78965*Z[2];
	Z[5] = 0.78965*Z[0];
	Z[6] = 0.165;
	M[5] = Z[14]-Z[6]*(Z[2]*(-(Z[10]+Z[4]))-Z[0]*(Z[7]+Z[5]))+0.0329+0.145*(Z[0]*Z[9]-Z[2]*Z[12]);
	Z[7] = 0.07526215029278779926*Z[2];
	Z[9] = -0.0274739834837783004*Z[0];
	Z[10] = Z[7]+Z[9]+0.07179454623553609659;
	Z[12] = 0.07526215029278779926*Z[0];
	Z[14] = -0.0274739834837783004*Z[2];
	Z[16] = Z[14]-Z[12]+0.03211993553327720254;
	Z[18] = Z[10]*Z[10]+Z[16]*Z[16];
	Z[19] = 0.04465499663169130187*Z[2];
	Z[22] = 2.219775824563799962e-09*Z[0];
	Z[23] = Z[22]+Z[19]-2.219790209527749963e-09;
	Z[33] = 0.04465499663169130187*Z[0];
	Z[36] = 2.219775824563799962e-09*Z[2];
	Z[37] = Z[36]-Z[33]-0.04465497329256169882;
	Z[38] = Z[23]*Z[23]+Z[37]*Z[37];
	Z[39] = Z[42]*Y[16];
	Z[41] = Z[41]*Y[12];
	Z[32] = Z[46]-Z[32]*Z[39]-Z[34]*Z[41];
	Z[28] = Z[28]*Z[39]+Z[30]*Z[41]+Z[45];
	if( Z[18]>1e-15 )
		Z[30] = Z[18];
	else
		Z[30] = 1e-15;
	if( Z[38]>1e-15 )
		Z[34] = Z[38];
	else
		Z[34] = 1e-15;
	if( Z[30]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	if( Z[34]<=0. ) {
		YP[0] = (*dsn_undef);
		return;
	}
	V[1] = Z[6]*(Z[2]*(Z[5]*Z[48]+Z[32]+Z[47])-Z[0]*(Z[4]*Z[48]+Z[28]+Z[49]))+Z[15]+Y[20]*Z[43]*(-0.1009578529976279903*Z[20]-0.0001612100150651070088*Z[17])+Z[31]*(-0.001196151814771630075*Z[24]-0.1001922804*Z[25])-Z[13]-Y[60]*Y[9]-Z[1]-Z[3]-Z[27]-Z[29]-(Y[28]*(Z[37]*(Z[22]+Z[19])-Z[23]*(Z[36]-Z[33])))/sqrt(Z[34])-((Z[16]*(Z[7]+Z[9])-Z[10]*(Z[14]-Z[12]))*Y[32])/sqrt(Z[30])+0.145*(Z[0]*(-(Z[28]+Z[11]+Z[49]))-Z[2]*(-(Z[8]+Z[47]+Z[32])));
	LSQDecompSolve(2,M,4,P,V);
	if( P[1]==0 ) {
		YP[0] = (*dsn_undef);
		return;
	}
	YP[7] = V[0];
	YP[9] = V[1];
	Z[1] = Y[54];
	if( Y[1]<Z[1] )
		Y[17] = 0.0075+0.0225*Y[1];
	else {
		Z[0] = 0.5+1.5*Y[1];
		Z[0] = 1.0/Z[0];
		Y[17] = 0.04*Z[0];
	}
	YP[0] = (1.*(Y[53]-Y[0]))/Y[13];
	Z[1] = Y[55];
	if( Y[2]<Z[1] )
		Y[21] = 0.0075+0.0225*Y[2];
	else {
		Z[0] = 0.5+1.5*Y[2];
		Z[0] = 1.0/Z[0];
		Y[21] = 0.04*Z[0];
	}
	YP[1] = (1.*(Y[54]-Y[1]))/Y[17];
	Z[1] = Y[56];
	if( Y[3]<Z[1] )
		Y[25] = 0.0075+0.0225*Y[3];
	else {
		Z[0] = 0.5+1.5*Y[3];
		Z[0] = 1.0/Z[0];
		Y[25] = 0.04*Z[0];
	}
	YP[2] = (1.*(Y[55]-Y[2]))/Y[21];
	Z[1] = Y[58];
	if( Y[5]<Z[1] )
		Y[33] = 0.0075+0.0225*Y[5];
	else {
		Z[0] = 0.5+1.5*Y[5];
		Z[0] = 1.0/Z[0];
		Y[33] = 0.04*Z[0];
	}
	YP[3] = (1.*(Y[56]-Y[3]))/Y[25];
	Z[1] = Y[57];
	if( Y[4]<Z[1] )
		Y[29] = 0.0075+0.0225*Y[4];
	else {
		Z[0] = 0.5+1.5*Y[4];
		Z[0] = 1.0/Z[0];
		Y[29] = 0.04*Z[0];
	}
	YP[5] = (1.*(Y[58]-Y[5]))/Y[33];
	YP[4] = (1.*(Y[57]-Y[4]))/Y[29];
	Y[34] = YP[7];
	Y[35] = YP[9];
}

static void otp(double T, double *Y, double *YP)
{
	double Z[8];

	UNUSED(T);
	UNUSED(YP);
	Y[36] = -Y[12];
	Y[37] = -Y[16];
	Y[38] = -Y[20];
	Y[39] = -Y[24];
	Y[40] = -Y[28];
	Y[41] = -Y[32];
	Z[0] = cos(Y[8]);
	Z[1] = cos(Y[6]);
	Z[2] = sin(Y[8]);
	Z[3] = sin(Y[6]);
	Z[4] = Z[0]*Z[1]-Z[2]*Z[3];
	Z[5] = 0.31;
	Z[6] = 0.34*Z[4];
	Z[7] = Z[0]*Z[5]+Z[6];
	Z[0] = Z[0]*Z[3]+Z[1]*Z[2];
	Z[1] = 0.34*Z[0];
	Z[2] = -(Z[2]*Z[5]+Z[1]);
	Z[3] = Y[9]*Y[9];
	Z[5] = Y[7]*Y[7];
	Y[42] = Y[35]*Z[7]+Z[2]*Z[3]-0.68*Z[0]*Y[7]*Y[9]+0.34*(Y[34]*Z[4]-Z[0]*Z[5]);
	Y[43] = 0.;
	Y[44] = Y[35]*Z[2]-Z[3]*Z[7]-0.68*Z[4]*Y[7]*Y[9]-0.34*(Y[34]*Z[0]+Z[4]*Z[5]);
	Y[45] = Z[6]*Y[7]+Z[7]*Y[9];
	Y[46] = 0.;
	Y[47] = Z[2]*Y[9]-Z[1]*Y[7];
	Y[48] = -Z[2];
	Y[49] = 0.;
	Y[50] = Z[7];
}

static void inpfn(double T, double *U)
{

}

static void SolverUpdate(double *u, int internal, SolverStruct *S)
{
	int i;

	inpfn(S->w[0],u);
	for(i=0;i<NINP;i++) S->w[i+NDIFF+NIX1-NINP+1]=u[i];
	fp(NEQ,S->w[0],&S->w[1],&S->w[NEQ+NPAR+1]);
	if(IS_UNDEF(S->w[NEQ+NPAR+1])) {
		SolverError(S,0,"index-1 and derivative evaluation failure");
		return;
	}
	if(internal) return;
	otp(S->w[0],&S->w[1],&S->w[NEQ+NPAR+1]);
	S->py[ 0]=S->w[ 7];
	S->py[ 1]=S->w[ 8];
	S->py[ 2]=S->w[ 9];
	S->py[ 3]=S->w[10];
	S->py[ 4]=S->w[43];
	S->py[ 5]=S->w[44];
	S->py[ 6]=S->w[45];
	S->py[ 7]=S->w[46];
	S->py[ 8]=S->w[47];
	S->py[ 9]=S->w[48];
	S->py[10]=S->w[49];
	S->py[11]=S->w[50];
	S->py[12]=S->w[51];
	S->py[13]=S->w[37];
	S->py[14]=S->w[38];
	S->py[15]=S->w[39];
	S->py[16]=S->w[40];
	S->py[17]=S->w[41];
	S->py[18]=S->w[42];
}

static void SolverOutputs(double *y, SolverStruct *S)
{
	otp(S->w[0],&S->w[1],&S->w[NEQ+NPAR+1]);
	y[ 0]=S->w[ 7];
	y[ 1]=S->w[ 8];
	y[ 2]=S->w[ 9];
	y[ 3]=S->w[10];
	y[ 4]=S->w[43];
	y[ 5]=S->w[44];
	y[ 6]=S->w[45];
	y[ 7]=S->w[46];
	y[ 8]=S->w[47];
	y[ 9]=S->w[48];
	y[10]=S->w[49];
	y[11]=S->w[50];
	y[12]=S->w[51];
	y[13]=S->w[37];
	y[14]=S->w[38];
	y[15]=S->w[39];
	y[16]=S->w[40];
	y[17]=S->w[41];
	y[18]=S->w[42];
}

static void RK4Step(double *u, SolverStruct *S)
{
	int i;
	double y[NEQ+1],yp1[NDFA],yp2[NDFA],yp3[NDFA];

	for(i=0;i<NEQ+1;i++) y[i]=S->w[i];
	for(i=0;i<NDIFF;i++) yp1[i]=S->w[1+NEQ+NPAR+i];
	S->w[0]+=0.5*S->h;
	for(i=0;i<NDIFF;i++) S->w[i+1]+=0.5*S->h*S->w[1+NEQ+NPAR+i];
	SolverUpdate(u,1,S);
	for(i=0;i<NDIFF;i++) yp2[i]=S->w[1+NEQ+NPAR+i];
	for(i=0;i<NDIFF;i++) S->w[i+1]=y[i+1]+0.5*S->h*S->w[1+NEQ+NPAR+i];
	SolverUpdate(u,1,S);
	for(i=0;i<NDIFF;i++) yp3[i]=S->w[1+NEQ+NPAR+i];
	S->w[0]=y[0]+S->h;
	for(i=0;i<NDIFF;i++) S->w[i+1]=y[i+1]+S->h*S->w[1+NEQ+NPAR+i];
	SolverUpdate(u,1,S);
	for(i=0;i<NDIFF;i++) S->w[i+1]=y[i+1]+S->h/6.0*(yp1[i]+2.0*(yp2[i]+yp3[i])+S->w[1+NEQ+NPAR+i]);
	SolverUpdate(u,0,S);
}

static void SolverSetup(double t0, double *ic, double *u, double *p, double *y, double h, SolverStruct *S)
{
	int i, j;

	S->h = h;
	S->iw=NULL;
	S->w[0] = t0;
	S->w[1] =  0.00000000000000000e+00;
	S->w[2] =  0.00000000000000000e+00;
	S->w[3] =  0.00000000000000000e+00;
	S->w[4] =  0.00000000000000000e+00;
	S->w[5] =  0.00000000000000000e+00;
	S->w[6] =  0.00000000000000000e+00;
	S->w[7] =  2.35619449019234484e+00;
	S->w[8] =  0.00000000000000000e+00;
	S->w[9] =  5.23598775598298816e-01;
	S->w[10] =  0.00000000000000000e+00;
	S->w[11] =  1.00000000000000000e+00;
	S->w[12] =  0.00000000000000000e+00;
	S->w[13] =  0.00000000000000000e+00;
	S->w[14] =  8.00000000000000017e-02;
	S->w[15] =  1.00000000000000000e+00;
	S->w[16] =  0.00000000000000000e+00;
	S->w[17] =  0.00000000000000000e+00;
	S->w[18] =  8.00000000000000017e-02;
	S->w[19] =  1.00000000000000000e+00;
	S->w[20] =  0.00000000000000000e+00;
	S->w[21] =  0.00000000000000000e+00;
	S->w[22] =  8.00000000000000017e-02;
	S->w[23] =  1.00000000000000000e+00;
	S->w[24] =  0.00000000000000000e+00;
	S->w[25] =  0.00000000000000000e+00;
	S->w[26] =  7.49999999999999972e-03;
	S->w[27] =  1.00000000000000000e+00;
	S->w[28] =  0.00000000000000000e+00;
	S->w[29] =  0.00000000000000000e+00;
	S->w[30] =  8.00000000000000017e-02;
	S->w[31] =  1.00000000000000000e+00;
	S->w[32] =  0.00000000000000000e+00;
	S->w[33] =  0.00000000000000000e+00;
	S->w[34] =  8.00000000000000017e-02;
	S->w[35] =  0.00000000000000000e+00;
	S->w[36] =  0.00000000000000000e+00;
	S->w[37] =  0.00000000000000000e-01;
	S->w[38] =  0.00000000000000000e-01;
	S->w[39] =  0.00000000000000000e-01;
	S->w[40] =  0.00000000000000000e-01;
	S->w[41] =  0.00000000000000000e-01;
	S->w[42] =  0.00000000000000000e-01;
	S->w[43] =  0.00000000000000000e+00;
	S->w[44] =  0.00000000000000000e+00;
	S->w[45] =  0.00000000000000000e+00;
	S->w[46] =  0.00000000000000000e-01;
	S->w[47] =  0.00000000000000000e+00;
	S->w[48] =  0.00000000000000000e-01;
	S->w[49] =  2.42998475334857078e-01;
	S->w[50] =  0.00000000000000000e+00;
	S->w[51] = -5.99469057651071946e-02;
	S->w[52] =  0.00000000000000000e+00;
	S->w[53] =  0.00000000000000000e+00;
	S->w[54] =  0.00000000000000000e+00;
	S->w[55] =  0.00000000000000000e+00;
	S->w[56] =  0.00000000000000000e+00;
	S->w[57] =  1.00000000000000006e-01;
	S->w[58] =  0.00000000000000000e+00;
	S->w[59] =  0.00000000000000000e+00;
	S->w[60] =  1.00000000000000000e+00;
	S->w[61] =  1.00000000000000000e+00;

	for(i=0;i<NDIFF;i++) S->w[i+NEQ+NPAR+1]=0.0;

	inpfn(S->w[0],u);
	for(i=0;i<NINP;i++) S->w[i+NDIFF+NIX1-NINP+1]=u[i];
	if(ic) for(i=0,j=0;i<NDIFF;i++) if(icvis[i]) { if(!IS_UNDEF(ic[j])) S->w[i+1]=ic[j]; j++; }
	if(p) for(i=0;i<NPAR;i++) S->w[i+NEQ+1]=p[i];
	S->w[NEQ+NPAR+1]=0.0;
	fp(NEQ,S->w[0],&S->w[1],&S->w[NEQ+NPAR+1]);
	if(IS_UNDEF(S->w[NEQ+NPAR+1])) {
		SolverError(S,0,"index-1 and derivative evaluation failure");
		return;
	}
	SolverOutputs(y,S);
}

/*
	Parametrized simulation driver
*/
#define EPT 1

#ifdef FROM_MAPLE
#include <time.h>

EXP long M_DECL ParamDriverMC(double t0, double dt, long npts, long stepsperpt, double *ic, double *p, double *out, char *errbuf, long internal, ALGEB halt)
#else
EXP long M_DECL ParamDriverMC(double t0, double dt, long npts, long stepsperpt, double *ic, double *p, double *out, char *errbuf, long internal)
#endif
{
	double u[NINP],y[NOUT+1];
	long i,j,k;
#ifdef FROM_MAPLE
	long ictr=0,pct=0;
	char buf[200];
	double tv1,tv2;
#endif
	SolverStruct S;

	/* Setup */
	for(j=0;j<=NOUT;j++) y[j]=0.0; /* Compiler warnings */
	for(i=0;i<(npts+EPT*(npts-1))*(NOUT+1);i++) out[i]=*dsn_undef;
	S.w=(double *)malloc((1+2*NEQ+NPAR+NDFA+NEVT)*sizeof(double));
	if(internal==0) S.err=0; else S.err=-1;
	S.buf=errbuf;
	SolverSetup(t0,ic,u,p,y,dt,&S);
	/* Output */
	out[0]=t0; for(j=0;j<NOUT;j++) out[j+1]=y[j];
	/* Integration loop */
#ifdef FROM_MAPLE
	tv1=clock()/(1.0*CLOCKS_PER_SEC);
#endif
	k=1;
	for(i=1;i<npts;i++) {
		for(j=0;j<stepsperpt;j++) {
			/* Take a step with states */
			RK4Step(u,&S);
			if( S.err>0 ) break;
#ifdef FROM_MAPLE
			ictr++;
			if(ictr>=100) {
				if( kv->getInterruptValue() ) {
					free(S.w);
					return(i);
				}
				tv2=clock()/(1.0*CLOCKS_PER_SEC);
				if(tv2-tv1>0.5) {
					if( halt!=NULL && !kv->isUnassignedName(halt) ) {
						free(S.w);
						return(i);
					}
					tv1=tv2;
				}
				ictr=0;
			}
#endif

		}
#ifdef FROM_MAPLE
		j=(100*i)/(npts-1);
		if(j>pct) {
			pct=j;
			sprintf(buf,"ProgressUpdate: %li %f",(long)pct,S.w[0]);
			kv->userinfo(1,"MapleSimProgressUpdate",buf);
		}
#endif
		/* Output */
#if EPT==1
		out[k*(NOUT+1)]=S.w[0]; for(j=0;j<NOUT;j++) out[k*(NOUT+1)+j+1]=S.py[j];
		k++;
#endif
		SolverOutputs(y,&S);
		out[k*(NOUT+1)]=S.w[0]; for(j=0;j<NOUT;j++) out[k*(NOUT+1)+j+1]=y[j];
		k++;
	}
#ifdef _SOLVER_TERMINATE
	if(i==npts)
		SolverTerminate(&S);
#endif

	free(S.w);
	return(k);
}

/*
	Old interface - maintain for backward compatibility
*/
EXP long M_DECL ParamDriverC(double t0, double dt, long npts, double *ic, double *p, double *out, char *errbuf, long internal)
{
#ifdef FROM_MAPLE
	ALGEB a=NULL;

	return ParamDriverMC(t0,dt,npts,1,ic,p,out,errbuf,internal,a);
#else
	return ParamDriverMC(t0,dt,npts,1,ic,p,out,errbuf,internal);
#endif
}

#ifdef FROM_MAPLE
EXP ALGEB M_DECL ParamDriver( MKernelVector kv_in, ALGEB *args )
{
	double t0,tf,dt,*ic,*p,*out;
	M_INT nargs,bounds[4],npts,naout,stepsperpt,sppa=0,i;
	RTableSettings s;
	ALGEB outd,halt;
	char buf[10000];

	kv=kv_in;
	nargs=kv->numArgs((ALGEB)args);
	if( nargs<5 || nargs>7 )
		kv->error("incorrect number of arguments");

	/* Process time vals */
	if( !kv->isNumeric(args[1]) )
		kv->error("argument #1, the initial time, must be numeric");
	t0=kv->mapleToFloat64(args[1]);
	if( !kv->isNumeric(args[2]) )
		kv->error("argument #2, the final time, must be numeric");
	tf=kv->mapleToFloat64(args[2]);
	if( t0>=tf )
		kv->error("the final time must be larger than the initial time");
	if( !kv->isNumeric(args[3]) )
		kv->error("argument #3, the time step, must be a positive numeric value");
	dt=kv->mapleToFloat64(args[3]);
	if(dt<=0)
		kv->error("argument #3, the time step, must be a positive numeric value");

	/* Check for presence of stepsperpt argument */
	if( kv->isInteger(args[4]) && (stepsperpt=kv->mapleToInteger32(args[4]))>0 )
		sppa=1;
	else
		stepsperpt=1;
	npts=(M_INT)floor((tf+1e-10*dt-t0)/(stepsperpt*dt))+1;

	/* Processing ic in */
	if( NDIFF==0 )
		ic=NULL;
	else if( kv->isInteger(args[4+sppa]) && kv->mapleToInteger32(args[4+sppa])==0 )
		ic=NULL;
	else if( !kv->isRTable(args[4+sppa]) ) {
		ic=NULL;
		kv->error("argument #%1, the initial data, must be a 1..ndiff rtable",kv->toMapleInteger(4+sppa));
	}
	else {
		kv->rtableGetSettings(&s,args[4+sppa]);
		if( s.storage != RTABLE_RECT || s.data_type != RTABLE_FLOAT64 ||
			 s.num_dimensions != 1 || kv->rtableLowerBound(args[4+sppa],1)!=1 ||
			 kv->rtableUpperBound(args[4+sppa],1) != NSDIFF )
			kv->error("argument #%1, the initial data, must be a 1..ndiff rtable",kv->toMapleInteger(4+sppa));
		ic=(double *)kv->rtableData(args[4+sppa]);
	}

	/* Processing parameters in */
	if( NPAR==0 )
		p=NULL;
	else if( kv->isInteger(args[5+sppa]) && kv->mapleToInteger32(args[5+sppa])==0 )
		p=NULL;
	else if( !kv->isRTable(args[5+sppa]) ) {
		p=NULL;
		kv->error("argument #%1, the parameter data, must be a 1..npar rtable",kv->toMapleInteger(5+sppa));
	}
	else {
		kv->rtableGetSettings(&s,args[5+sppa]);
		if( s.storage != RTABLE_RECT || s.data_type != RTABLE_FLOAT64 ||
			 s.num_dimensions != 1 || kv->rtableLowerBound(args[5+sppa],1)!=1 ||
			 kv->rtableUpperBound(args[5+sppa],1) != NPAR )
			kv->error("argument #%1, the parameter data, must be a 1..npar rtable",kv->toMapleInteger(5+sppa));
		p=(double *)kv->rtableData(args[5+sppa]);
	}

	/* Output data table */
	if( nargs==6+sppa ) {
		outd=NULL;
		if( !kv->isRTable(args[6+sppa]) ) {
			out=NULL;
			naout=0;
			kv->error("argument #%1, the output data, must be a 1..npts,1..nout+1 C_order rtable",kv->toMapleInteger(6+sppa));
		}
		else {
			kv->rtableGetSettings(&s,args[6+sppa]);
			if( s.storage != RTABLE_RECT || s.data_type != RTABLE_FLOAT64 ||
			 	s.order != RTABLE_C || s.num_dimensions != 2 ||
			 	kv->rtableLowerBound(args[6+sppa],1)!=1 ||
			 	kv->rtableLowerBound(args[6+sppa],2)!=1 ||
			 	kv->rtableUpperBound(args[6+sppa],2) != NOUT+1 )
				kv->error("argument #%1, the output data, must be a 1..npts,1..nout+1 C_order rtable",kv->toMapleInteger(6+sppa));
			naout=kv->rtableUpperBound(args[6+sppa],1);
			if( naout<1 )
				kv->error("argument #%1, the output data, must have at least 1 output slot",kv->toMapleInteger(6+sppa));
			out=(double *)kv->rtableData(args[6+sppa]);
			if(naout<npts+EPT*(npts-1)) npts=(EPT+naout)/(EPT+1);
		}
	}
	else {
		kv->rtableGetDefaults(&s);
		bounds[0]=1; bounds[1]=npts+EPT*(npts+1);
		bounds[2]=1; bounds[3]=NOUT+1;
		s.storage=RTABLE_RECT;
		s.data_type=RTABLE_FLOAT64;
		s.order=RTABLE_C;
		s.num_dimensions=2;
		s.subtype=RTABLE_ARRAY;
		outd=kv->rtableCreate(&s,NULL,bounds);
		out=(double *)kv->rtableData(outd);
		naout=npts+EPT*(npts+1);
	}
	for(i=0;i<naout*(NOUT+1);i++) out[i]=*dsn_undef;

	halt=kv->toMapleName("_HALT_MAPLESIM_INTEGRATION",TRUE);

	i=ParamDriverMC(t0,dt,npts,stepsperpt,ic,p,out,buf,1,halt);

	/* All done */
	if(outd==NULL)
		return(kv->toMapleInteger(i));
	else
		return(outd);
}
#endif
