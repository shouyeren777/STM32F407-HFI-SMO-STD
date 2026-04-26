
#include "observer_drv.h"  

/**
 * 函数功能: 滑模观测器（SMO）核心计算
 * 输入参数: p - SMO_STRUCT结构体指针，包含电机参数（Rs、Ld）、采样周期Ts、电压Ualpha/Ubeta、
 *           电流Ialpha/Ibeta、滑模增益Gain、滤波系数EabForeLPFFactor，以及观测器状态变量
 * 返回参数: 无（结果存入p->IalphaFore, p->IbetaFore, p->EalphaFore, p->EbetaFore,
 *           p->EalphaForeLPF, p->EbetaForeLPF）
 * 说    明: 
 *         1. 滑模观测器原理：
 *            基于永磁同步电机在αβ坐标系下的电压方程：
 *              di_alpha/dt = (-Rs/Ld)*i_alpha + (Ualpha - Ealpha)/Ld
 *              di_beta/dt  = (-Rs/Ld)*i_beta  + (Ubeta  - Ebeta)/Ld
 *            其中Ealpha、Ebeta为反电动势，包含转子速度和位置信息。
 *            通过构造电流观测器，利用滑模切换函数迫使观测电流跟踪实际电流，
 *            从而提取反电动势观测值。
 *         2. 算法步骤：
 *            (1) 电流预测：基于上一时刻的观测电流和反电动势（滤波后），
 *                使用前向欧拉法计算当前观测电流IalphaFore、IbetaFore。
 *            (2) 滑模面切换：计算电流误差 (IalphaFore - Ialpha)，通过饱和函数
 *                得到估计反电动势EalphaFore、EbetaFore。饱和函数采用边界层
 *                （±1.0）设计，在边界层内线性输出，外部输出±Gain，有效抑制抖振。
 *            (3) 低通滤波：对滑模输出的高频开关信号进行一阶低通滤波，
 *                得到平滑的反电动势基波分量EalphaForeLPF、EbetaForeLPF，
 *                用于下一周期的电流预测和位置/速度估算。
 *         3. 关键参数说明：
 *            - Rs：定子电阻，需通过辨识或参数表获得。
 *            - Ld：直轴电感（对表贴式电机Ld=Lq）。
 *            - Gain：滑模增益，须大于反电动势幅值，通常根据最大转速下的反电动势设定。
 *            - EabForeLPFFactor：低通滤波系数（0~1），取值小则滤波效果好但相位滞后大，
 *              需在噪声抑制和动态响应间权衡。
 *            - 边界层阈值1.0f：根据电流量程标幺化设定，实际使用时可能需要调整。
 *         4. 输出用途：
 *            滤波后的反电动势EalphaForeLPF、EbetaForeLPF送入锁相环（PLL）
 *            即可估算转子角度和角速度，实现无位置传感器控制。
 *         5. 注意事项：
 *            - 滑模观测器在中高速段表现良好，低速段信噪比低，估算精度下降。
 *            - 需确保电机参数Rs、Ld准确，否则观测器可能发散。
 *            - 实际应用中常加入反电动势归一化或自适应增益调整。
 */
void SMO_Calculate(SMO_STRUCT *p)
{
    // 1. 电流预测（基于电机模型的前向欧拉离散化）
    // 公式：I_fore(k+1) = I_fore(k) + Ts * [ -Rs/Ld * I_fore(k) + (U - E_foreLPF)/Ld ]
    p->IalphaFore += p->Ts * (-p->Rs / p->Ld * p->IalphaFore + (p->Ualpha - p->EalphaForeLPF) / p->Ld);
    p->IbetaFore  += p->Ts * (-p->Rs / p->Ld * p->IbetaFore  + (p->Ubeta  - p->EbetaForeLPF)  / p->Ld);

    // 2. 滑模切换函数（饱和函数，减小抖振）
    // 边界层设定为 ±1.0（可根据实际电流误差范围调整）
    // 当误差 > 1.0 时，输出 +Gain
    if ((p->IalphaFore - p->Ialpha) > 1.0f)
        p->EalphaFore = p->Gain;
    // 当误差 < -1.0 时，输出 -Gain
    else if ((p->IalphaFore - p->Ialpha) < -1.0f)
        p->EalphaFore = -p->Gain;
    // 在边界层内，输出 = Gain * 误差（线性区）
    else
        p->EalphaFore = p->Gain * (p->IalphaFore - p->Ialpha);

    // β轴同理
    if ((p->IbetaFore - p->Ibeta) > 1.0f)
        p->EbetaFore = p->Gain;
    else if ((p->IbetaFore - p->Ibeta) < -1.0f)
        p->EbetaFore = -p->Gain;
    else
        p->EbetaFore = p->Gain * (p->IbetaFore - p->Ibeta);

    // 3. 反电动势低通滤波（滤除高频切换噪声）
    // 一阶低通滤波：E_filt(k) = α * E_raw(k) + (1-α) * E_filt(k-1)
    p->EalphaForeLPF = p->EalphaFore * p->EabForeLPFFactor + p->EalphaForeLPF * (1 - p->EabForeLPFFactor);
    p->EbetaForeLPF  = p->EbetaFore  * p->EabForeLPFFactor + p->EbetaForeLPF  * (1 - p->EabForeLPFFactor);
}

/**
 * 函数功能: 高频注入（HFI）及信号解析（用于零低速无传感器控制）
 * 输入参数: p - HFI_STRUCT结构体指针，包含高频注入的电压幅值、频率、电流采样值、
 *           滤波系数、NSD状态标志、累加和、输出方向等
 * 返回参数: 无（结果存入p->NSDOut（磁极极性判断）、p->IalphaOut/p->IbetaOut（高频响应包络）等）
 * 说    明: 
 *         1. 高频注入法适用于永磁同步电机零速和低速时的转子初始位置辨识。
 *            基本原理：向d轴注入高频旋转电压信号，通过提取高频电流响应的幅值
 *            和相位信息，解算转子位置和磁极极性。
 *         2. 该函数分为两大步骤：
 *            (1) 初始角度辨识（NSD，North-South Detection）：通过施加正负d轴电流
 *                脉冲，比较高频电流响应幅值，判断磁极极性（N/S）。
 *            (2) 信号注入与解析：提取αβ轴高频电流分量，并经过差分处理得到
 *                包络信号，用于后续锁相环（PLL）计算转子位置。
 *         3. NSD算法流程（状态机基于p->NSDCount）：
 *            - 0~399次：IdRef=0，等待观测器/滤波器收敛。
 *            - 400~599次：施加正向IdRef=5A，建立正向磁场。
 *            - 600~609次：保持正向电流，同时累加高频电流幅值到NSDSum1。
 *            - 610~809次：IdRef=0，等待电流归零。
 *            - 810~1009次：施加负向IdRef=-5A，建立反向磁场。
 *            - 1010~1019次：保持负向电流，累加高频电流幅值到NSDSum2。
 *            - 1020次：IdRef=0。
 *            - 1021次：比较NSDSum2与NSDSum1，若后者大则NSDOut=1（表示某极性），
 *              否则NSDOut=0。通过极性判断确定转子N极方向。
 *         4. 信号注入与解析：
 *            - 通过前后两次采样求平均提取基频分量IdBase、IqBase（用于电流环）。
 *            - 利用相邻采样点差值提取αβ轴高频电流分量IalphaHigh、IbetaHigh。
 *            - 交替改变IalphaOut/IbetaOut的计算方向（基于Dir标志），得到高频包络信号，
 *              该信号与转子位置误差成正比，送入PLL即可获取转子角度。
 *         5. 适用场景：零速和低速（通常<额定转速5%）时使用，中高速切换至滑模观测器。
 */
void HFI_Calculate(HFI_STRUCT *p)
{
    /* ================= 第一部分：初始角度辨识（南北极检测） ================= */
    if (p->NSDFlag == 0)
    {
        p->NSDCount++;   // 状态计数器递增

        // 提取d轴高频电流分量：相邻两次Id采样差的一半
        p->IdHigh = (p->Id - p->IdLast) * 0.5f;
        if (p->IdHigh < 0)
        {
            p->IdHigh = -p->IdHigh;   // 取绝对值（幅值）
        }

        // 状态机：根据计数值执行不同操作
        if (p->NSDCount < 10400)
        {
            // 阶段0：等待观测器/滤波器收敛（约400次调用）
            p->IdRef = 5.0f;
        }
        else if (p->NSDCount >= 10400 && p->NSDCount < 10600)
        {
            // 阶段1：施加正向电流脉冲（+5A）
            p->IdRef = 5.0f;
        }
        else if (p->NSDCount >= 10600 && p->NSDCount < 10610)
        {
            // 阶段2：保持正向电流，同时采样高频电流幅值（累加10次）
            p->IdRef = 5.0f;
            p->NSDSum1 += p->IdHigh;
        }
        else if (p->NSDCount >= 10610 && p->NSDCount < 10810)
        {
            // 阶段3：等待电流归零
            p->IdRef = 0.0f;
        }
        else if (p->NSDCount >= 10810 && p->NSDCount < 11010)
        {
            // 阶段4：施加负向电流脉冲（-5A）
            p->IdRef = -5.0f;
        }
        else if (p->NSDCount >= 11010 && p->NSDCount < 11020)
        {
            // 阶段5：保持负向电流，采样高频电流幅值（累加10次）
            p->IdRef = -5.0f;
            p->NSDSum2 += p->IdHigh;
        }
        else if (p->NSDCount == 11020)
        {
            // 阶段6：电流归零
            p->IdRef = 0.0f;
        }
        else if (p->NSDCount == 11021)
        {
            // 阶段7：完成NSD，比较正负脉冲下的高频电流响应幅值
            p->NSDFlag = 1;       // NSD完成标志
            p->NSDCount = 0;      // 计数器复位

            // 若负向电流下的高频幅值大于正向，则NSDOut=1，否则0
            // 该结果用于确定转子N极相对于注入轴的方向
            if (p->NSDSum2 > p->NSDSum1)
            {
                p->NSDOut = 1;
            }
            else
            {
                p->NSDOut = 0;
            }
        }
    }

    /* ================= 第二部分：信号注入与解析（高频响应解调） ================= */
    // 提取d、q轴基频电流分量（相邻两次采样平均，滤除高频成分）
    p->IdBase = (p->Id + p->IdLast) * 0.5f;
    p->IqBase = (p->Iq + p->IqLast) * 0.5f;
    // 更新上次采样值
    p->IdLast = p->Id;
    p->IqLast = p->Iq;

    // 保存上一周期的高频电流分量（用于差分）
    p->IalphaHighLast = p->IalphaHigh;
    p->IbetaHighLast  = p->IbetaHigh;

    // 提取αβ轴高频电流分量：相邻两次采样差的一半
    // 由于高频注入电压频率远高于基频，相邻采样点之差主要反映高频响应
    p->IalphaHigh = (p->Ialpha - p->IalphaLast) * 0.5f;
    p->IbetaHigh  = (p->Ibeta  - p->IbetaLast)  * 0.5f;

    // 更新上次αβ电流采样值
    p->IalphaLast = p->Ialpha;
    p->IbetaLast  = p->Ibeta;

    // 交替计算高频包络信号（用于PLL解调位置误差）
    // 通过Dir标志交替改变差分方向，可消除直流偏移，增强信噪比
    if (p->Dir == 0)
    {
        // 当前周期输出 = 本次高频分量 - 上次高频分量
        p->IalphaOut = p->IalphaHigh - p->IalphaHighLast;
        p->IbetaOut  = p->IbetaHigh  - p->IbetaHighLast;
        p->Dir = 1;   // 翻转方向，下次计算相反差分
    }
    else if (p->Dir == 1)
    {
        // 输出 = 上次高频分量 - 本次高频分量
        p->IalphaOut = p->IalphaHighLast - p->IalphaHigh;
        p->IbetaOut  = p->IbetaHighLast  - p->IbetaHigh;
        p->Dir = 0;
    }
}

/**
 * 函数功能: 正交锁相环（QPLL）—— 从正交信号中提取转子角度和角速度
 * 输入参数: p - PLL_STRUCT结构体指针，包含输入正交信号Ain、Bin（通常为反电动势或高频响应），
 *           当前观测角度的正余弦值CosVal、SinVal，方向标志Dir，比例系数Kp、积分系数Ki，
 *           采样周期Ts，角速度滤波系数WeForeLPFFactor，以及输出状态变量
 * 返回参数: 无（结果存入p->ThetaErr（角度误差）、p->WeFore（观测角速度）、
 *           p->WeForeLPF（滤波后角速度）、p->ThetaFore（观测电角度）、
 *           p->ETheta（0~4095格式角度）、p->EThetaPU（标幺化角度））
 * 说    明: 
 *         1. 正交锁相环原理：
 *            输入为两相正交信号（例如滑模观测器输出的Ealpha、Ebeta，或高频注入的包络信号）。
 *            对于反电动势，有：Ealpha = -Em * sinθ，Ebeta = Em * cosθ。
 *            构造误差函数：err = cosθ_hat * Ealpha + sinθ_hat * Ebeta。
 *            代入得 err = -Em * (cosθ_hat sinθ - sinθ_hat cosθ) = -Em * sin(θ_hat - θ)。
 *            当θ_hat ≈ θ时，err ≈ -Em * (θ_hat - θ)，即误差与角度差成正比。
 *            通过PI控制器调节角速度，使误差趋于零，从而实现角度跟踪。
 *         2. 算法步骤：
 *            (1) 计算角度误差ThetaErr = Dir * (CosVal * Ain + SinVal * Bin)。
 *                Dir为旋转方向（±1），用于适应正反转，使误差符号一致。
 *            (2) PI调节器：比例项PPart = Kp * ThetaErr，积分项IPart累加 Ki * ThetaErr，
 *                输出角速度WeFore = PPart + IPart。
 *            (3) 对角速度进行低通滤波，得到WeForeLPF，减小噪声。
 *            (4) 积分得到角度：ThetaFore += WeFore * Ts。
 *            (5) 角度归一化到[0, 2π)范围。
 *            (6) 角度格式转换：ETheta = ThetaFore * (4095 / 2π) ≈ ThetaFore * 651.7395，
 *                用于与正余弦查表或编码器接口匹配；EThetaPU = ETheta / 4095，即标幺值（0~1）。
 *         3. 关键参数：
 *            - Kp、Ki：决定锁相环的带宽和响应速度。带宽通常设为注入频率或反电动势基频的1/10~1/5。
 *            - Ts：锁相环执行周期，需与采样周期一致。
 *            - WeForeLPFFactor：角速度低通滤波系数（0~1），取值小则速度平滑但响应慢。
 *         4. 输出用途：
 *            - ThetaFore（或转换后的EThetaPU）可作为无传感器控制下的转子电角度，
 *              用于Park/反Park变换，实现闭环控制。
 *         5. 注意事项：
 *            - 输入信号Ain、Bin需经过归一化或幅值稳定，否则误差增益会变化。
 *            - 锁相环对正交信号的相位和幅值敏感，输入信号质量直接影响估算精度。
 *            - 方向标志Dir需根据电机旋转方向设定，可由速度观测器或启动逻辑提供。
 */
void PLL_Calculate(PLL_STRUCT *p)
{
    // 1. 计算角度误差（锁相环鉴相器）
    // 公式：ThetaErr = Dir * (cosθ_hat * Ain + sinθ_hat * Bin)
    // 当Ain、Bin为Ealpha、Ebeta时，该误差近似正比于角度差
    p->ThetaErr = p->Dir * (p->CosVal * p->Ain + p->SinVal * p->Bin);

    // 2. PI控制器（环路滤波器）
    p->PPart = p->Kp * p->ThetaErr;            // 比例项
    p->IPart += p->Ki * p->ThetaErr;           // 积分项累加
    p->WeFore = p->PPart + p->IPart;           // 观测角速度（rad/s）

    // 3. 角速度低通滤波（可选，用于平滑输出）
    p->WeForeLPF = p->WeFore * p->WeForeLPFFactor + p->WeForeLPF * (1 - p->WeForeLPFFactor);

    // 4. 积分得到观测电角度（压控振荡器）
    p->ThetaFore += p->WeFore * p->Ts;          // 角度积分

    // 5. 角度归一化到 [0, 2π)
    if (p->ThetaFore > 6.28318f)               // 2π ≈ 6.28318
    {
        p->ThetaFore -= 6.28318f;
    }
    else if (p->ThetaFore < 0)
    {
        p->ThetaFore += 6.28318f;
    }

    // 6. 角度格式转换（供外部使用）
    p->ETheta = p->ThetaFore;
    // 标幺化角度 0~1 对应 0~2π
    p->EThetaPU = p->ETheta / 6.28318f;
}

    






