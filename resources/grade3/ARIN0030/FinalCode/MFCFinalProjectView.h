
// MFCFinalProjectView.h: CMFCFinalProjectView 类的接口
//

#pragma once

#include "CDlgSetArg.h"
#include<iostream>
#include<cmath>
#include<thread>
#include<algorithm>
#include<vector>
#include<set>
#include<iomanip>
#include<functional>
#include<cstdlib>

#include <optional>
#include <variant>
#include <any>
using namespace std;
static const double PI = 3.14;
static const double g = 9.8;
static const double minControl = -50.0;
static const double maxControl = 50.0;

static const double ToForce = 7.0;




class PIDSolver {
private:

	double h;
	double dt;
	
	int sz;
	double* m_curve;
	function<double(double, double, double)> func;
public:
	double Force;
	double Kp = 2.5;
	double Ki = 0.1;
	double Kd = 0.5;
	double m = 1.0;
	double mg = m * g;
	double f = 1.0;
	double k = 1.0;
	PIDSolver();
	~PIDSolver() { 
		//delete[]m_curve;
	}
	PIDSolver(const PIDSolver& S) :h(S.h), dt(S.dt), Force(S.Force), sz(S.sz) {
		m_curve = new double[sz];
		size_t num = sz * sizeof(double);
		memcpy(m_curve, S.m_curve, num);
		SetFunc();
	}
	PIDSolver(double duration) :dt(duration), h(duration / 10) {
		SetProperDt();
		h = dt / 10;
		Force = -ToForce;
		sz = dt/h;
		m_curve = new double[sz];
		SetFunc();
	}
	PIDSolver(double hx, double tx) :h(hx), dt(tx) {
		SetProperDt();
		if (tx / hx < 5) {
			hx = tx / 5;
		}
		Force = -ToForce;
		sz = floor(tx / hx);
		m_curve = new double[sz];
		SetFunc();
	}
	double getDh() {
		return this->dt;
	}
	void SetProperDt();
	void SetKArgs(double k1, double k2, double k3);
	void SetFunc();
	void SetValK(double k1, double k2, double k3) {
		Kp = k1;
		Ki = k2;
		Kd = k3;
	}
	void SetValPhy(double mx, double fx, double kx) {
		m = mx;
		mg = m * g;
		f = fx;
		k = kx;
	}
	vector<double> getCurve();

	void PIDController(double tarPos, double curPos, double& integral, double& preError);
	//double RogeKuttaOde2(double t_0, double t_end, double x_0, double v_0);
	pair<double, vector<double>> RogeKuttaOde2(double t_0, double x_0, double v_0, function<double(double, double, double)>f = nullptr);


	//void debugInfo() {
	//	cout << "kp is" << Kp << endl;
	//}

};

class CMFCFinalProjectView : public CView
{
protected: // 仅从序列化创建
	CMFCFinalProjectView() noexcept;
	DECLARE_DYNCREATE(CMFCFinalProjectView)

// 特性
public:
	CMFCFinalProjectDoc* GetDocument() const;
	afx_msg void OnButtonStart();
	afx_msg void OnButtonStop();
	afx_msg void OnButtonSusp();
	;
	;
	;

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// 实现
public:
	virtual ~CMFCFinalProjectView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()

public:
	/*double m;
	double mg;
	double f;
	double k;*/
	double base;
	double valA;
	double valW;

	double targetPos;


	int flag;
	double duration1;
	double duration2;
	double duration3;

	int m_susp;
	int m_stop;
	int m_start;
	//double Kp;
	//double KI;
	//double Kd;

	//friend class PIDSolver;

	PIDSolver m_pid;
	//CCriticalSection m_cs;

	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnOptDlg();
};



#ifndef _DEBUG  // MFCFinalProjectView.cpp 中的调试版本
inline CMFCFinalProjectDoc* CMFCFinalProjectView::GetDocument() const
   { return reinterpret_cast<CMFCFinalProjectDoc*>(m_pDocument); }
#endif

