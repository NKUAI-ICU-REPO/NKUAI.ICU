#pragma once


// CDlgSetArg 对话框

class CDlgSetArg : public CDialog
{
	DECLARE_DYNAMIC(CDlgSetArg)

public:
	CDlgSetArg(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~CDlgSetArg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_SET };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	double val_g;
	double val_f;
	double val_k;
	CEdit editBase;
	double val_b;
	CEdit editOmega;
	double val_w;
	double val_A;
	int m_curve;
	double val_p;
	double val_I;
	double val_d;
	afx_msg void OnBnClickedOk();
	afx_msg void OnChangeArgA();
	afx_msg void OnChangeArgB();
	afx_msg void OnChangeArgD();
	afx_msg void OnChangeArgF();
	afx_msg void OnChangeArgI();
	afx_msg void OnChangeArgK();
	afx_msg void OnChangeArgP();
	afx_msg void OnChangeArgM();
	afx_msg void OnChangeArgW();
	afx_msg void OnBnClickedRadioStep();
	afx_msg void OnBnClickedRadioSin();
};
