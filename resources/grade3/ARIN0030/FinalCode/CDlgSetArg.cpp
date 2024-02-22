// CDlgSetArg.cpp: 实现文件
//

#include "pch.h"
#include "MFCFinalProject.h"
#include "CDlgSetArg.h"
#include "afxdialogex.h"


// CDlgSetArg 对话框

IMPLEMENT_DYNAMIC(CDlgSetArg, CDialog)

CDlgSetArg::CDlgSetArg(CWnd* pParent /*=nullptr*/)
	: CDialog(IDD_DIALOG_SET, pParent)
	, val_g(1.0)
	, val_f(1.0)
	, val_k(1.0)
	, val_b(2.0)
	, val_w(3.14)
	, val_A(2.0)
	, m_curve(0)
	, val_p(2.5)
	, val_I(0.1)
	, val_d(0.5)
{

}

CDlgSetArg::~CDlgSetArg()
{
}

void CDlgSetArg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_ARG_M, val_g);
	DDV_MinMaxDouble(pDX, val_g, 1, 3);
	DDX_Text(pDX, IDC_ARG_F, val_f);
	DDV_MinMaxDouble(pDX, val_f, 1, 5);
	DDX_Text(pDX, IDC_ARG_K, val_k);
	DDV_MinMaxDouble(pDX, val_k, 1, 5);
	DDX_Control(pDX, IDC_ARG_B, editBase);
	DDX_Text(pDX, IDC_ARG_B, val_b);
	DDV_MinMaxDouble(pDX, val_b, 0, 2);
	DDX_Control(pDX, IDC_ARG_W, editOmega);
	DDX_Text(pDX, IDC_ARG_W, val_w);
	DDV_MinMaxDouble(pDX, val_w, 1.24, 6.28);
	DDX_Text(pDX, IDC_ARG_A, val_A);
	DDX_Radio(pDX, IDC_RADIO_SIN, m_curve);
	DDV_MinMaxInt(pDX, m_curve, 0, 1);
	DDX_Text(pDX, IDC_ARG_P, val_p);
	DDX_Text(pDX, IDC_ARG_I, val_I);
	DDX_Text(pDX, IDC_ARG_D, val_d);
}


BEGIN_MESSAGE_MAP(CDlgSetArg, CDialog)
	
	ON_BN_CLICKED(IDOK, &CDlgSetArg::OnBnClickedOk)
	ON_EN_CHANGE(IDC_ARG_A, &CDlgSetArg::OnChangeArgA)
	ON_EN_CHANGE(IDC_ARG_B, &CDlgSetArg::OnChangeArgB)
	ON_EN_CHANGE(IDC_ARG_D, &CDlgSetArg::OnChangeArgD)
	ON_EN_CHANGE(IDC_ARG_F, &CDlgSetArg::OnChangeArgF)
	ON_EN_CHANGE(IDC_ARG_I, &CDlgSetArg::OnChangeArgI)
	ON_EN_CHANGE(IDC_ARG_K, &CDlgSetArg::OnChangeArgK)
	ON_EN_CHANGE(IDC_ARG_P, &CDlgSetArg::OnChangeArgP)
	ON_EN_CHANGE(IDC_ARG_M, &CDlgSetArg::OnChangeArgM)
	ON_EN_CHANGE(IDC_ARG_W, &CDlgSetArg::OnChangeArgW)
	ON_BN_CLICKED(IDC_RADIO_STEP, &CDlgSetArg::OnBnClickedRadioStep)
	ON_BN_CLICKED(IDC_RADIO_SIN, &CDlgSetArg::OnBnClickedRadioSin)
END_MESSAGE_MAP()


// CDlgSetArg 消息处理程序

void CDlgSetArg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialog::OnOK();
}



void CDlgSetArg::OnChangeArgA()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgB()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgD()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgF()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgI()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgK()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgP()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgM()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}


void CDlgSetArg::OnChangeArgW()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialog::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
}




void CDlgSetArg::OnBnClickedRadioStep()
{
	GetDlgItem(IDC_STATIC_MODEL)->ShowWindow(SW_NORMAL);
	editBase.ShowWindow(SW_HIDE);
	editOmega.ShowWindow(SW_HIDE);
	m_curve = 1;
	//editBase.SetWindowTextW(_T("0"));
	//editOmega.SetWindowTextW(_T("0"));
	editBase.EnableWindow(FALSE);
	editOmega.EnableWindow(FALSE);
}


void CDlgSetArg::OnBnClickedRadioSin()
{
	GetDlgItem(IDC_STATIC_MODEL)->ShowWindow(SW_NORMAL);
	editBase.ShowWindow(SW_NORMAL);
	editOmega.ShowWindow(SW_NORMAL);
	m_curve = 0;
	editBase.EnableWindow();
	editOmega.EnableWindow();
	// TODO: 在此添加控件通知处理程序代码
}
