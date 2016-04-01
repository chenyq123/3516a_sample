#include <math.h>

static void gauss_solve(int n,double A[],double x[],double b[])
{
	int i,j,k,r;
	double max;
	for (k = 0; k < n-1; k++)	{
		max = fabs(A[k*n+k]); /*find maxmum*/
		r = k;
		for (i = k + 1; i < n-1; i++) {
			if (max < fabs(A[i*n+i])) {
				max = fabs(A[i*n+i]);
				r=i;
			}
		}

		if (r != k) {
			for (i = 0; i < n; i++) /*change array:A[k]&A[r] */	{
				max = A[k*n+i];
				A[k*n+i] = A[r*n+i];
				A[r*n+i] = max;
			}
		}

		max = b[k]; /*change array:b[k]&b[r] */
		b[k] = b[r];
		b[r] = max;
		for (i = k+1; i<n; i++)	{
			for (j = k+1; j < n; j++)
				A[i*n+j] -= A[i*n+k] * A[k*n+j] / A[k*n+k];
			b[i] -= A[i*n+k] * b[k] / A[k*n+k];
		}
	}

	for (i = n-1; i >= 0; x[i] /= A[i*n+i], i--) {
		for (j = i+1, x[i] = b[i]; j<n; j++) {
			x[i] -= A[i*n+j] * x[j];
		}
	}
}

//*==================polyfit(n,x,y,poly_n,a)===================*/
//*=======拟合y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n========*/
//*=====n是数据个数 xy是数据值 poly_n是多项式的项数======*/
//*===返回a0,a1,a2,……a[poly_n]，系数比项数多一（常数项）=====*/

void polyfit(int n, double x[], double y[], int poly_n, double a[])
{
	int i,j;
	double *tempx, *tempy, *sumxx, *sumxy, *ata;

	tempx = new double[n];
	sumxx = new double[poly_n*2+1];
	tempy = new double[n];
	sumxy = new double[poly_n+1];
	ata = new double[(poly_n+1)*(poly_n+1)];

	/**	x,y 为测量点数据，n 数目大于 poly_n+1，

		相当于求解 poly_n + 1 个解，但有 n 个方程
		Ax=y 
		
		对于这个很可能没解的方程，如何求出它的近似解呢？利用线性代数的思维来思考，思路非常直观。我们知道对于所有可能的向量x，Ax的取值
		构成了一个向量空间，叫做矩阵A的列空间，A的列空间中的所有向量都是矩阵A的列向量的线性和。(注：可以看作Ax张成完备空间中的一个子空
		间，求近似解释就相当于求y在该子空间中的投影的解）方程Ax=y无解当且仅当向量y不在A的列空间中。既然y不在A的列空间，要求得近似解，
		很自然的想法是在A的列空间中找到一个离y最近的点y'，将方程变为：

			Ax = y'

		立体几何知识告诉我们，对于平面外的一点p，平面上离它最近的点，是点p在该平面上的正交投影点。即通过点p向平面做垂线，垂线与平面的交
		点，即为该平面上离点p最近的点。这个结论在高维空间中依然成立。向量(y-y')是点y与点y'的连线，所以该连线与A的列空间垂直，用向量的
		语言表达就是：

			对A的列空间中的任意向量Av：(Av)T(y-y') = 0

			化简一下：vTAT(y-y') = 0。

			由于v是任意向量，一个与任意向量点乘都等于0的向量，必然是零向量（想一下该向量与自己点乘，结果依然为0），所以AT(y-y')=0。
			事实上，如果对A的四个基本子空间比较了解的话，这个式子的得来会更加直接：因为A的列空间与AT的零空间（即ATx=0的解空间）为互补
			正交子空间，所有与A的列空间正交的向量都在AT的零空间中，所以AT(y-y')=0。

		将y' = Ax代入并化简：

			ATAx = ATy

		当Ax=y无解时，对这个式子求解就能得到x的近似解，这就是著名的最小二乘法。

		例如，假如平面上有三个不共线的点(1,1)，(2,2)，(3,2)，要求得一条近似穿过这三个点的直线y=c1+c2x，就是求解方程：

			XTXc=XTy

		其中X=[1 1; 1 2; 1 3]，y=[1;2;2]
		可以解出系数向量c = (XTX)-1XTy = [0.6667;0.5]

		多项式拟合，可以认为是一组方程，求解 a0, a1, a2 ... 
		y0 = a0 + a1x1 + a2x1*x1 + a3x1*x1*x1 + ...
					||
					|| 
					|| 
					\/  
			y   =   A                    a

				  [ 1 x1 x1^2 x1^3 ... ][a0]
 			[y] = [ 1 x2 x2^2 x2^3 ... ][a1]
				  [ .......            ][.]
				  
			其中 [y] [x1, x2, ...] 已知，求解 [a0, a1, ...]

			因为 系数矩阵有可能不可逆，两面都乘上系数矩阵的转置
				ATy = ATAa
			ATA 往往可逆
				ATAa = ATy	求解 a 向量
	 */

	for (i = 0; i < n; i++) {
		tempx[i] = 1;
		tempy[i] = y[i];
	}

	for (i = 0; i < 2*poly_n + 1; i++) {
		sumxx[i] = 0;
		for (j = 0; j < n; j++) {
			sumxx[i] += tempx[j];
			tempx[j] *= x[j];
		}
	}

	for (i = 0; i < poly_n+1; i++) {
		sumxy[i] = 0;
		for (j = 0; j < n; j++) {
			sumxy[i] += tempy[j];
			tempy[j] *= x[j];
		}
	}

	// ATA
	for (i = 0; i < poly_n+1; i++) {
		for (j = 0; j < poly_n+1; j++) {
			ata[i * (poly_n + 1) + j] = sumxx[i + j];
		}
	}

	gauss_solve(poly_n+1, ata , a, sumxy);

	delete []tempx;
	delete []sumxx;
	delete []tempy;
	delete []sumxy;
	delete []ata;
}
