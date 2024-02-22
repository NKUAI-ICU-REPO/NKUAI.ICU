
// MFCFinalProjectView.cpp: CMFCFinalProjectView 类的实现
//

#include "pch.h"
#include "framework.h"
// SHARED_HANDLERS 可以在实现预览、缩略图和搜索筛选器句柄的
// ATL 项目中进行定义，并允许与该项目共享文档代码。
#ifndef SHARED_HANDLERS
#include "MFCFinalProject.h"
#endif

#include "MFCFinalProjectDoc.h"
#include "MFCFinalProjectView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMFCFinalProjectView

IMPLEMENT_DYNCREATE(CMFCFinalProjectView, CView)

BEGIN_MESSAGE_MAP(CMFCFinalProjectView, CView)
	// 标准打印命令
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
    ON_WM_TIMER()
    ON_COMMAND(ID_OPT_DLG, &CMFCFinalProjectView::OnOptDlg)
    ON_COMMAND(ID_BUTTON_START, &CMFCFinalProjectView::OnButtonStart)
    ON_COMMAND(ID_BUTTON_STOP, &CMFCFinalProjectView::OnButtonStop)
    ON_COMMAND(ID_BUTTON_SUSP, &CMFCFinalProjectView::OnButtonSusp)
END_MESSAGE_MAP()



double ih1, iw1, ih2, iw2, ih3, iw3;
double w1, w2, w3;
double h1, h2, h3;
double Perror = 0.0;
double Vzero = 0.0;
double Xzero = 0.0;
double Integral = 0.0;

double InitPos;
double InitKnotY;

// CMFCFinalProjectView 构造/析构

CMFCFinalProjectView::CMFCFinalProjectView() noexcept
{
	// TODO: 在此处添加构造代码
    /*Kp = 1.0;
    KI = 1.0;
    Kd = 1.0;*/

    base = 0.0;
    valW = PI;
    valA = 4.3;

 /*   m = 1.0;
    mg = m * g;
    f = 1.0;
    k = 1.0;*/


    flag = 1;

     m_pid = PIDSolver();
     m_susp = 0;

     targetPos = valA;

     duration1 = 0.0;
     duration2 = 0.0;
     duration3 = 0.0;


     Xzero = (m_pid.mg + m_pid.Force) / m_pid.k;
     InitPos = Xzero;
     Perror = valA - Xzero+0.1;

    //OutputDebugString(_T("Kp is %d", m_pid.Kp));


}

PIDSolver::PIDSolver() {
    Force = -ToForce;
    dt = 0.1;
    h = 0.01;
    sz = dt/h;
    m_curve = new double[sz];
    SetFunc();
}
void PIDSolver::SetKArgs(double k1, double k2, double k3) {
    Kp = k1;
    Ki = k2;
    Kd = k3;
}
void PIDSolver::SetProperDt() {
    dt = (dt > 0.2 || dt<0.0) ? 0.1 : dt;
}
void PIDSolver::PIDController(double tarPos, double curPos, double& integral, double& preError) {
    double error = tarPos - curPos;
     

    integral += error * dt;
    double derivate = (error - preError) / dt;

    double control = Kp * error + Ki * integral + Kd * derivate;
    preError = error;

    Force = control-mg+k*curPos;
    SetFunc();

}
void PIDSolver::SetFunc() {

    func = [&](double t, double x, double v) ->double {
        return (mg+Force - f * v - k * x) / m;
    };
}
pair<double,vector<double>> PIDSolver::RogeKuttaOde2(double t_0, double x_0, double v_0, function<double(double, double, double)>f) {
    if (f == nullptr) {
        f = func;
    }
    vector<double>topic;
    double t = t_0;
    double x = x_0;
    double v = v_0;

    double t_end = t + h * sz; 
    int cnt = 0;
    while (t < t_end) {
        double k1x = h * v;
        double k1v = h * f(t,x,v);

        double k2x = h * (v + k1v / 2.0);
        double k2v = h * f(t + h / 2.0, x + k1x / 2.0, v + k1v / 2.0);

        
        double k3x = h * (v + k2v / 2.0);
        double k3v = h * f(t + h / 2.0, x + k2x / 2.0, v + k2v / 2.0);
        
        double k4x = h * (v + k3v);
        double k4v = h * f(t + h, x + k3x, v + k3v);


        x += (k1x + 2.0 * k2x + 2.0 * k3x + k4x) / 6.0;
        v += (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
        

        t += h;
        m_curve[cnt++] = x;
    }
    topic.push_back(x);
    topic.push_back(v);
    auto ans = make_pair(t_end, topic);
    return ans;
}
vector<double> PIDSolver::getCurve() {
    vector<double>ans;
    for (int i = 0; i < sz; i++) {
        ans.push_back(m_curve[i]);
    }
    return ans;
}
CMFCFinalProjectView::~CMFCFinalProjectView()
{

  

}

BOOL CMFCFinalProjectView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

// CMFCFinalProjectView 绘图
vector<double> SetAndPaintGca(CDC*pDC,double interval,double width,double height,double top_height,double bottom_height,
    double rulerW,double rulerH,double Widmeter,double Heimeter,bool flag=true) {

    vector<double>s;
    CBrush NewBrush;
    CBrush* pOldBrush;

    CPen NewPen;
    CPen ColPen;
    CPen cpen;

    CPen* pOldPen;


    NewBrush.CreateSolidBrush(RGB(0, 0, 0));
  
  
    

    ColPen.CreatePen(PS_SOLID, 3, RGB(255, 165, 0));
    pOldPen = pDC->SelectObject(&ColPen);
    if(flag)
        pDC->Rectangle(-interval-5, top_height+5, width - interval+5, bottom_height-5);
    else
        pDC->Rectangle(-interval- 5, top_height -5, width - interval + 5, bottom_height +5);

    pDC->SelectObject(pOldPen);


    pOldBrush = pDC->SelectObject(&NewBrush);
    pDC->Rectangle(-interval, top_height, width - interval, bottom_height);
    pDC->SelectObject(pOldBrush);

    NewPen.CreatePen(PS_SOLID, 3, RGB(255, 255, 255));
    cpen.CreatePen(PS_SOLID, 2, RGB(255, 0, 0));

    pOldPen = pDC->SelectObject(&NewPen);

 
    

        pDC->MoveTo(-interval, 0);
        pDC->LineTo(width, 0);

  
        pDC->MoveTo(0, bottom_height);
        pDC->LineTo(0, top_height);



        int sign = bottom_height / abs(bottom_height);
        double bth = sign>0?bottom_height-6:bottom_height+6;
        if (flag) {
            pDC->MoveTo(0, bth);
            pDC->LineTo(-5, bth + 5);

            pDC->MoveTo(0, bth);
            pDC->LineTo(5, bth + 5);
        }
        else {
            pDC->MoveTo(0, bth);
            pDC->LineTo(-5, bth -5);

            pDC->MoveTo(0, bth);
            pDC->LineTo(5, bth - 5);
        }
    pDC->MoveTo(width - interval-5, 0);
    pDC->LineTo(width - interval - 8, -4);
    pDC->MoveTo(width - interval, 0);
    pDC->LineTo(width - interval -8, 4);
    pDC->SelectObject(pOldPen);

    double interWid =( abs(width-interval)-20)/ rulerW;
    double interHei = abs(top_height - bottom_height) / (rulerH+1);

    double tmpwidth = interWid;
    double tmpheight;
    double cnt;
   
    if (flag) {
     cnt = -floor(rulerH/2)*Heimeter;
   
     tmpheight = top_height - interHei * rulerH / 20.0;
    }
    else {
        cnt = Heimeter;
        tmpheight = top_height+interHei * rulerH / 10.0;
    }
    CString strTmp;



    pOldPen = pDC->SelectObject(&NewPen);

    pDC->SetBkColor(RGB(0, 0, 0));
    pDC->SetTextColor(RGB(255, 255, 255));

    if (flag) {

        while (tmpheight >= bottom_height+ interHei * rulerH / 20.0) {
            
            pDC->MoveTo(0, tmpheight);
            pDC->LineTo(-10, tmpheight);
            strTmp.Format(_T("%.2f"), cnt);
            pDC->TextOut(-interval + 10, tmpheight, strTmp);
            cnt += Heimeter;
           /* if (tmpheight - interHei < 0.0 && tmpheight>0.0) {
                tmpheight = 0.0;
                continue;
            }*/
            tmpheight -= interHei;
        }
    }
    else {
        while (tmpheight <= bottom_height-interHei * rulerH / 10.0) {
            pDC->MoveTo(0, tmpheight);
            pDC->LineTo(-10, tmpheight);
            strTmp.Format(_T("%.2f"), cnt);
            pDC->TextOut(-interval + 10, tmpheight, strTmp);
            cnt -= Heimeter;
           /* if (tmpheight +interHei >0.0&&tmpheight<0.0) {
                tmpheight = 0.0;
                continue;
            }*/
            tmpheight += interHei;
        }
    }
    pDC->SelectObject(pOldPen);

    pOldPen = pDC->SelectObject(&NewPen);

    cnt = Widmeter;
    while (tmpwidth <= abs(width-interval-20)) {
        pDC->MoveTo(tmpwidth, 0);
        pDC->LineTo(tmpwidth, -10);
        strTmp.Format(_T("%.2f"), cnt);
        pDC->TextOut(tmpwidth, -interval + 10, strTmp);
        cnt += Widmeter;
        tmpwidth += 1 * interWid;
    }

    pDC->SelectObject(pOldPen);

    s.push_back(interWid);
    s.push_back(interHei);
    return s;

    //if (!flag) {
    //    XFORM xform;
    //    auto angle = asin(-1);
    //    xform.eM11 = cos(angle);   // x 轴方向的缩放因子为 cos(angle)
    //    xform.eM12 = sin(angle);   // x 轴方向的旋转因子为 sin(angle)
    //    xform.eM21 = -sin(angle);  // y 轴方向的旋转因子为 -sin(angle)
    //    xform.eM22 = cos(angle);   // y 轴方向的缩放因子为 cos(angle)
    //    xform.eDx = 0;
    //    xform.eDy = 0;

    //    pDC->SetWorldTransform(&xform);
    //}

}
void CMFCFinalProjectView::OnDraw(CDC* pDC)
{
	CMFCFinalProjectDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;



    // 恢复绘图参数


    CRect rect;
    GetClientRect(&rect);


    //CPoint ptrZero(80, rect.Height() / 2.0-50);

    CPoint ptrZero(80, rect.Height() / 4.0+20);


    pDC->DPtoLP(&ptrZero);

    pDC->SetWindowOrg(-ptrZero.x, -ptrZero.y);


   
    double width = rect.Width() / 2.0 - 220;
    double  height = rect.Height() / 2.0-20;
   // CClientDC dc(this);

    double interval = 50;
   // double top_height = 50;
    double top_height = height/2-30;
    //double bottom_height=-height+80;

    double bottom_height = -height / 2 + 30;

    double rulerW = 8.0;
    double rulerH = 12.0;
    double Widmeter = 0.5;
    double Heimeter = 0.9;


    w1 = Widmeter;
    h1 = Heimeter;

    auto s=SetAndPaintGca(pDC, interval, width, height, top_height, bottom_height, rulerW, rulerH, Widmeter, Heimeter);
    iw1 = s[0];
    ih1 = s[1];

    ptrZero.y = rect.Height()/2+90;

    //ptrZero.y = rect.Height()*3 / 4.0 +20;


    pDC->SetWindowOrg(0, 0);
    pDC->DPtoLP(&ptrZero);
    pDC->SetWindowOrg(-ptrZero.x, -ptrZero.y);
    top_height = -50;
    bottom_height = height-90;
    rulerH = 12.0;
    Heimeter = 3.6;
    w2 = Widmeter;
    h2 = Heimeter;

    s=SetAndPaintGca(pDC, interval, width, height, top_height, bottom_height, rulerW, rulerH, Widmeter, Heimeter,false);
    iw2 = s[0];
    ih2 = s[1];
    
  /*  ptrZero.x = 160 + rect.Width() / 3;
    ptrZero.y = rect.Height() / 2.0 - 50;*/


    ptrZero.x = 160 + rect.Width() / 3;
    ptrZero.y = rect.Height() / 4.0+10;

    pDC->SetWindowOrg(0, 0);
    pDC->DPtoLP(&ptrZero);
    pDC->SetWindowOrg(-ptrZero.x, -ptrZero.y);

    top_height = height / 2 - 30;
    bottom_height = -height/2+30;
    rulerH = 10.0;
    Heimeter = 0.9;
    w3 = Widmeter;
    h3 = Heimeter;


    s=SetAndPaintGca(pDC, interval, width, height, top_height, bottom_height, rulerW, rulerH, Widmeter, Heimeter);

    iw3 = s[0];
    ih3 = s[1];

  
    CPen penSpring,penTarget;
    CPen* pOldPen;
    penSpring.CreatePen(PS_SOLID, 3, RGB(0, 0, 0));
    penTarget.CreatePen(PS_SOLID, 3, RGB(255, 0, 0));

    ptrZero.x=- 120 + rect.Width();
    ptrZero.y = 100;

    pDC->SetWindowOrg(0, 0);
    pDC->DPtoLP(&ptrZero);
    pDC->SetWindowOrg(-ptrZero.x, -ptrZero.y);


    pOldPen = pDC->SelectObject(&penSpring);
    pDC->MoveTo(-40,0);
    pDC->LineTo(40, 0);

    double totalHei = rect.Height()*3/8;
    double initHei = totalHei / 20;
    double KnotY=0.0;

    while (initHei <= totalHei) {
        double KnotX = sqrt(pow(25, 2) - pow(totalHei / 20, 2));
        pDC->MoveTo(0, KnotY);
        pDC->LineTo(KnotX, initHei);
        pDC->MoveTo(KnotX, initHei);
        KnotY += totalHei / 10;
        pDC->LineTo(0, KnotY);
        initHei += totalHei / 10;
    }
    pDC->Rectangle(-15, KnotY, 15, KnotY + 30);
    pDC->SelectObject(pOldPen);

    double MapTar = KnotY / Xzero * targetPos;

    InitKnotY = KnotY;
    pOldPen = pDC->SelectObject(&penTarget);
    pDC->MoveTo(-30, MapTar);
    pDC->LineTo(30, MapTar);
    pDC->SelectObject(pOldPen);






	// TODO: 在此处为本机数据添加绘制代码
}


//UINT ThreadPaintPosition(LPVOID pParm) {
//
//
//    auto pMView = (CMFCFinalProjectView*)pParm;
//    auto dc = pMView->GetDC();
//
//    CPen pen1, pen2;
//    CPen* pOldPen;
//    pen1.CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
//    pen2.CreatePen(PS_SOLID, 2, RGB(0, 0, 255));
//
//
//    CRect rect;
//    pMView->GetClientRect(&rect);
//
//    CPoint ptrZero(80, rect.Height() / 2.0 - 50);
//    dc->DPtoLP(&ptrZero);
//    dc->SetWindowOrg(-ptrZero.x, -ptrZero.y);
//
//    auto valA = pMView->valA;
//    auto valW = pMView->valW;
//    auto base = pMView->base;
//    auto duration1 = pMView->duration1;
//    auto flag = pMView->flag;
//
//    Xzero = (pMView->m_pid.mg + pMView->m_pid.Force) / pMView->m_pid.k;
//    Perror = valA - Xzero;
//    double r;
//
//    if (flag == 0) {
//        double stamp = pMView->duration1;
//        double xTmp = Xzero;
//        double vTmp = Vzero;
//        pMView->m_pid.PIDController(valA, Xzero, Integral, Perror);
//        auto xy = pMView->m_pid.RogeKuttaOde2(duration1, Xzero, Vzero);
//
//
//        auto Dt = pMView->m_pid.getDh();
//        pMView->duration1 += Dt;
//        Xzero = xy.second[0];
//        Vzero = xy.second[1];
//
//
//        r = stamp / iw1;
//        dc->MoveTo(stamp, -(pMView->valA * sin(valW * r) + base));
//        r = (duration1) / iw1;
//        dc->LineTo(duration1, -(valA * sin(valW * r) + base));
//        //r2 = duration1 / iw1;
//        dc->MoveTo(stamp, -xTmp);
//        dc->LineTo(duration1, -Xzero);
//        if (duration1 >= 2.0) {
//            pMView->KillTimer(1);
//        }
//    }
//    else if (flag == 1) {
//        double stamp = duration1;
//        double xTmp = Xzero;
//        double vTmp = Vzero;
//        pMView->m_pid.PIDController(valA, Xzero, Integral, Perror);
//        auto xy = pMView->m_pid.RogeKuttaOde2(duration1, Xzero, Vzero);
//
//
//        auto Dt = pMView->m_pid.getDh();
//        duration1 += Dt;
//        Xzero = xy.second[0];
//        Vzero = xy.second[1];
//
//
//        //r = stamp / iw1;
//        dc->MoveTo(stamp, -valA);
//        dc->LineTo(duration1, -(valA));
//        //r2 = duration1 / iw1;
//        dc->MoveTo(stamp, xTmp);
//        dc->LineTo(duration1, Xzero);
//        if (duration1 >= 2.0) {
//            pMView->KillTimer(1);
//        }
//        
//    }
//    return 0;
//}

// CMFCFinalProjectView 打印

BOOL CMFCFinalProjectView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 默认准备
	return DoPreparePrinting(pInfo);
}

void CMFCFinalProjectView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加额外的打印前进行的初始化过程
}

void CMFCFinalProjectView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 添加打印后进行的清理过程
}


// CMFCFinalProjectView 诊断

#ifdef _DEBUG
void CMFCFinalProjectView::AssertValid() const
{
	CView::AssertValid();
}

void CMFCFinalProjectView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CMFCFinalProjectDoc* CMFCFinalProjectView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMFCFinalProjectDoc)));
	return (CMFCFinalProjectDoc*)m_pDocument;
}
#endif //_DEBUG


// CMFCFinalProjectView 消息处理程序

//struct ThreadParams
//{
//    CDC* pDC;  // 设备指针
//    // 其他线程函数所需的参数
//};



void CMFCFinalProjectView::OnTimer(UINT_PTR nIDEvent)
{

    if (nIDEvent == 1) {
        CClientDC dc(this);

        CPen pen1, pen2,pen3,pen4;
        CPen*pOldPen;
        pen1.CreatePen(PS_SOLID, 2, RGB(255, 0, 0));
        pen2.CreatePen(PS_SOLID, 2, RGB(0, 0, 255));
        pen3.CreatePen(PS_SOLID, 2, RGB(0, 255, 0));
        pen4.CreatePen(PS_SOLID,2,RGB(255, 215, 0));


        CRect rect;
        GetClientRect(&rect);

       // CPoint ptrZero(80, rect.Height() / 2.0 - 50);
        CPoint ptrZero(80, rect.Height() / 4.0 + 20);
        dc.DPtoLP(&ptrZero);
        dc.SetWindowOrg(-ptrZero.x, -ptrZero.y);

      
        //double ErrorTmp = Perror;
        //double r;
        double stamp = duration1;
        double xTmp = Xzero;
        double vTmp = Vzero;
        double TmpForce = m_pid.Force;
        if (!flag) {
            auto Dt = m_pid.getDh();
            duration1 += Dt;
            m_pid.PIDController(valA * sin(valW *stamp) + base, xTmp, Integral, Perror);
            auto xy = m_pid.RogeKuttaOde2(stamp, xTmp, Vzero);


           
            Xzero = xy.second[0];
            Vzero = xy.second[1];

            //dc.TextOutW(0, 80, _T("xzero is %.2f"),Xzero);


            //r = stamp / iw1;
            pOldPen = dc.SelectObject(&pen1);
            dc.MoveTo(stamp*iw1/w1, -(valA * sin(valW * stamp) + base)*ih1/h1);
            dc.LineTo(duration1*iw1/w1, -(valA * sin(valW * duration1) + base)*ih1/h1);
            dc.SelectObject(pOldPen);

            dc.SetBkColor(RGB(0, 0, 0));
            dc.SetTextColor(RGB(255, 0, 0));
            dc.TextOutW(300, -80, _T("目标曲线"));

            targetPos = valA * sin(valW * duration1) + base;
            
            //r2 = duration1 / iw1;
            pOldPen = dc.SelectObject(&pen2);
            dc.MoveTo(stamp*iw1/w1, -xTmp*ih1/h1);
            dc.LineTo(duration1*iw1/w1,-Xzero*ih1/h1);
            dc.SelectObject(pOldPen);
            dc.SetTextColor(RGB(0, 0, 255));
            dc.TextOutW(300, -60, _T("实际曲线"));
        }

        else if(flag==1) {
          
            m_pid.PIDController(valA, Xzero, Integral, Perror);
            auto xy = m_pid.RogeKuttaOde2(duration1, Xzero, Vzero);


            auto Dt = m_pid.getDh();
            duration1 += Dt;
            Xzero = xy.second[0];
            Vzero = xy.second[1];


            //r = stamp / iw1;
            pOldPen = dc.SelectObject(&pen1);
            dc.MoveTo(stamp*iw1/w1, -valA*ih1/h1);
            dc.LineTo(duration1*iw1/w1, -valA*ih1/h1);
            dc.SelectObject(pOldPen);

            dc.SetBkColor(RGB(0,0, 0));
            dc.SetTextColor(RGB(255, 0, 0));
            dc.TextOutW(300, -80, _T("目标曲线"));

            targetPos = valA;
            //r2 = duration1 / iw1;
            pOldPen = dc.SelectObject(&pen2);
            dc.MoveTo(stamp*iw1/w1, -xTmp*ih1/h1);
            dc.LineTo(duration1*iw1/w1, -Xzero*ih1/h1);
            dc.SelectObject(pOldPen);

            dc.SetTextColor(RGB(0, 0, 255));
            dc.TextOutW(300, -60, _T("实际曲线"));


        }


        ptrZero.y = rect.Height() / 2 +90;

        //ptrZero.y = rect.Height() * 3 / 4.0 + 20;

        dc.SetWindowOrg(0, 0);
        dc.DPtoLP(&ptrZero);
        dc.SetWindowOrg(-ptrZero.x, -ptrZero.y);

        pOldPen = dc.SelectObject(&pen3);
        dc.MoveTo(stamp*iw2/w2, -TmpForce*ih2/h2);
        dc.LineTo(duration1*iw2/w2,-m_pid.Force*ih2/h2);
        dc.SelectObject(pOldPen);
        dc.SetTextColor(RGB(0, 245, 0));
        dc.TextOutW(300, 180, _T("控制力曲线"));

       /* ptrZero.x = 160 + rect.Width() / 3;
        ptrZero.y = rect.Height() / 2.0 - 50;*/


        ptrZero.x = 160 + rect.Width() / 3;
        ptrZero.y = rect.Height() / 4.0 +10;

        dc.SetWindowOrg(0, 0);
        dc.DPtoLP(&ptrZero);
       dc.SetWindowOrg(-ptrZero.x, -ptrZero.y);

       pOldPen = dc.SelectObject(&pen4);
       dc.MoveTo(stamp*iw3/w3, -Perror*ih3/h3);
       double curError;
       if (flag) {
           curError = (valA - Xzero);
       }
       else {
           curError = valA * sin(valW * duration1) + base - Xzero;
       }
       dc.LineTo(duration1*iw3/w3, - curError*ih3/h3);
       dc.SelectObject(pOldPen);
        

       dc.SetTextColor(RGB(255, 215, 0));
       dc.TextOutW(300, -80, _T("误差曲线"));

       CPen penSpring, penTarget;
       //CPen* pOldPen;
       penSpring.CreatePen(PS_SOLID, 3, RGB(0, 0, 0));
       penTarget.CreatePen(PS_SOLID, 3, RGB(255, 0, 0));



       ptrZero.x = -120 + rect.Width();
       ptrZero.y = 100;

       

       dc.SetWindowOrg(0, 0);
       auto AreaToDraw = CRect(rect.Width() - 180, 100, rect.Width() - 60, rect.Height());
       //InvalidateRect(AreaToDraw);
       dc.FillSolidRect(AreaToDraw, GetSysColor(COLOR_WINDOW));
       dc.DPtoLP(&ptrZero);
       dc.SetWindowOrg(-ptrZero.x, -ptrZero.y);


       pOldPen = dc.SelectObject(&penSpring);
       dc.MoveTo(-40, 0);
       dc.LineTo(40, 0);

       double totalHei = rect.Height()*3 / (8*InitPos) * Xzero;
       double initHei = totalHei / 20;
       double KnotY = 0.0;

       while (initHei <= totalHei) {
           double KnotX = sqrt(pow(25, 2) - pow(totalHei / 20, 2));
           dc.MoveTo(0, KnotY);
           dc.LineTo(KnotX, initHei);
           dc.MoveTo(KnotX, initHei);
           KnotY += totalHei / 10;
           dc.LineTo(0, KnotY);
           initHei += totalHei / 10;
       }
       dc.Rectangle(-15, KnotY, 15, KnotY + 30);
       dc.SelectObject(pOldPen);
       double MapTar= InitKnotY / InitPos * targetPos;
       pOldPen = dc.SelectObject(&penTarget);
       dc.MoveTo(-30, MapTar);
       dc.LineTo(30, MapTar);
       dc.SelectObject(pOldPen);



        if (duration1 >= 4.2) {
            KillTimer(1);
        }

    }
    // TODO: 在此添加消息处理程序代码和/或调用默认值
 /*   else if (nIDEvent == 2) {

    }
    else if (nIDEvent == 3) {

    }*/
    CView::OnTimer(nIDEvent);
}


void CMFCFinalProjectView::OnOptDlg()
{


    CDlgSetArg Dlg;

    if (Dlg.DoModal() == IDOK) {
        valA = Dlg.val_A;
        valW = Dlg.val_w;
        base = Dlg.val_b;
        flag = Dlg.m_curve;
        m_pid.Kp = Dlg.val_p;
        m_pid.Kd = Dlg.val_d;
        m_pid.Ki = Dlg.val_I;
        m_pid.m = Dlg.val_g;
        m_pid.k = Dlg.val_k;
        m_pid.f = Dlg.val_f;
        m_pid.Force = -ToForce;
        m_pid.mg = m_pid.m * g;
        Integral = 0.0;

        Xzero = (m_pid.mg + m_pid.Force) / m_pid.k;
        Perror = valA - Xzero;

    }


    // TODO: 在此添加命令处理程序代码
}


void CMFCFinalProjectView::OnButtonStart()
{

    SetTimer(1, 100, NULL);

    // TODO: 在此添加命令处理程序代码
}


void CMFCFinalProjectView::OnButtonStop()
{
    KillTimer(1);


    duration1 = 0.0;
    m_pid.Force = -ToForce;
    Xzero = (m_pid.mg + m_pid.Force) / m_pid.k;
    Perror = valA - Xzero+0.1;
    Vzero = 0.0;
    Invalidate();
    // TODO: 在此添加命令处理程序代码
}


void CMFCFinalProjectView::OnButtonSusp()
{

    if (m_susp) {
        SetTimer(1, 100, NULL);
        m_susp = 0;

    }
    else {
        KillTimer(1);
        m_susp = 1;
    }
    // TODO: 在此添加命令处理程序代码
}
