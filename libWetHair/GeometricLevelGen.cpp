//
// This file is part of the libWetHair open source project
//
// Copyright 2015 Xinxin Zhang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "GeometricLevelGen.h"


//#define AMG_VERBOSE

template<class T>
void levelGen<T>::generateRP(const FixedSparseMatrix<T> &A,
	FixedSparseMatrix<T> &R,
	FixedSparseMatrix<T> &P,
	int ni, int nj, int nk)
{
	//matlab code(2D):
	//m = ceil(M/2);
	//n = ceil(N/2);
	//R = sparse(zeros(m*n, M*N));
	//coef = [1 3 3 1; 3 9 9 3; 3 9 9 3; 1 3 3 1]/16.0;
	//for i=0:m-1
	//	for j=0:n-1
	//		index = i*n + j +1;
	//		for ii=0:1
	//			for jj=0:1
	//				iii = i*2+ii;
	//				jjj = j*2+jj;
	//				if iii<M && jjj<N && iii>=0 && jjj>=0
	//					index2 = iii*N + jjj +1;
	//					if A(index2,index2)~=0
	//						R(index, index2) = 0.25;
	//						%R(index, index2) = coef(ii+2,jj+2)/4;
	//					else
	//						R(index, index2) = 0.0;
	//					end
	//				end
	//			end
	//		end
	//	end
	//end

	//P = 4*R';

	int nni = ceil((float)ni/2.0);
	int nnj = ceil((float)nj/2.0);
	int nnk = ceil((float)nk/2.0);
	SparseMatrix<T> r;
	SparseMatrix<T> p;
	p.resize(ni*nj*nk);
	p.zero();
	r.resize(nni*nnj*nnk);
	r.zero();

	for(int k=0;k<nnk;k++)for(int j=0;j<nnj;j++)for (int i=0;i<nni;i++)
	{
		unsigned int index = (k*nnj +j)*nni + i;
		for(int kk=0;kk<=1;kk++)for(int jj=0;jj<=1;jj++)for(int ii=0;ii<=1;ii++)
		{
			int iii = i*2 + ii;
			int jjj = j*2 + jj;
			int kkk = k*2 + kk;
			if(iii<ni && jjj<nj && kkk<nk)
			{
				unsigned int index2 = (kkk*nj + jjj)*ni + iii;
				r.set_element(index, index2, (T)0.125);
				p.set_element(index2,index, 1.0);
			}
		}
	}

	R.construct_from_matrix(r);
	P.construct_from_matrix(p);
	r.clear();
	p.clear();

	//transposeMat(R,P,(T)8.0);


}


template<class T>
void levelGen<T>::generateLevelsGalerkinCoarsening(vector<FixedSparseMatrix<T> > &A_L,
	vector<FixedSparseMatrix<T> > &R_L,
	vector<FixedSparseMatrix<T> > &P_L,
	vector<Vector3i> &S_L, int & total_level, 
	FixedSparseMatrix<T> &A, 
	int ni,int nj,int nk)
{
#ifdef AMG_VERBOSE
	cout<<"building levels ...... "<<endl;
#endif
	A_L.resize(0);
	R_L.resize(0);
	P_L.resize(0);
	S_L.resize(0);
	total_level = 1;
  A_L.push_back(std::move(A));
	S_L.push_back(Vector3i(ni,nj,nk));
	int nni = ni, nnj = nj, nnk = nk;
	unsigned int unknowns = ni*nj*nk;
	while (unknowns > 16*16*16)
	{
		A_L.push_back(FixedSparseMatrix<T>());
		R_L.push_back(FixedSparseMatrix<T>());
		P_L.push_back(FixedSparseMatrix<T>());
		nni = ceil((float)nni/2.0);
		nnj = ceil((float)nnj/2.0);
		nnk = ceil((float)nnk/2.0);

		S_L.push_back(Vector3i(nni,nnj,nnk));
		unknowns = nni*nnj*nnk;
		total_level++;
	}
	
	for (int i=0;i<total_level-1;i++)
	{
		generateRP((A_L[i]), (R_L[i]),(P_L[i]),S_L[i][0], S_L[i][1],S_L[i][2]);
		FixedSparseMatrix<T> temp;
		multiplyMat((A_L[i]),(P_L[i]),temp,1.0);
		multiplyMat((R_L[i]),temp, (A_L[i+1]),0.5);
		temp.resize(0);
		temp.clear();
	}
#ifdef AMG_VERBOSE
	cout<<"build levels done"<<endl;
#endif
}

template struct levelGen<double>;
