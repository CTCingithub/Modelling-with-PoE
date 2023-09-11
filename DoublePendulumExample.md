<!--
 * @Author: CTC 2801320287@qq.com
 * @Date: 2023-09-06 15:54:42
 * @LastEditors: CTC 2801320287@qq.com
 * @LastEditTime: 2023-09-12 00:19:27
 * @Description: Double pendulum dynamics derived with PoE
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
-->
# Double Pendulum Example

We attempt to model a double pendulum system under control with PoE. The system is shown as follows.

![The double pendulum system](images/01.jpg)

The inertia of two pendulums are $J_{1}$ and $J_{2}$ respectively. We set the x-axis vertically downwards and the y-axis horizontally to the right, and determined the z-axis using the right-hand rule.

Applying Jacobian matrixes and the second Lagrange theorem, it's easy to obtain dynamics of any robotic systems as follows:

$$
M(\vec{q}) \ddot{\vec{q}} + \underbrace{B(\vec{q}) \vec{\dot{q}\dot{q}} + C(\vec{q}, \dot{\vec{q}}) \vec{\dot{q}^{2}}}_{V \left( \vec{q},\dot{\vec{q}} \right)} + G(\vec{q}) = \tau_{d}.
$$

Where $B(\vec{q}) \vec{\dot{q}\dot{q}}$ represents Coriolis force, $C(\vec{q}, \dot{\vec{q}}) \vec{\dot{q}^{2}}$ represents centrifugal force, $G(\vec{q})$ represents gravity.

In this double-pendulum case, the special column vectors $\vec{\dot{q}\dot{q}}$ and $\vec{\dot{q}^{2}}$ follows:

$$
\vec{q} = \begin{pmatrix}
    \theta_{1} \\
    \theta_{2}
\end{pmatrix}, \qquad
\vec{\dot{q}\dot{q}} = \begin{pmatrix}
    \dot{\theta}_{1} \dot{\theta}_{2}
\end{pmatrix}, \qquad
\vec{\dot{q}^{2}} = \begin{pmatrix}
    \theta_{1}^{2} \\
    \theta_{2}^{2}
\end{pmatrix}.
$$

We define $J_{i}$ to be the Jacobian matrix for the $i$-th rigid body, which is a function of $\vec{q}$. Jacobian matrixes can be obtained with PoE, as their columns are unit twists of their corresponding rigid bodies. $J_{m_{1}}$ and $J_{m_{2}}$ in this case can be derived as is shown below:

$$
\hat{\omega}_{1} = \hat{\omega}_{2} = \begin{pmatrix}
    0 \\ 0 \\ 1
\end{pmatrix}, \\
\begin{aligned}
    e^{[\hat{\omega}_{1}] \theta_{1}} &= I + [\hat{\omega}_{1}] \sin{\theta_{1}} + [\hat{\omega}_{1}]^{2} (1 - \cos{\theta_{1}}) \\
    &= \begin{bmatrix}
        1 & 0 & 0 \\
        0 & 1 & 0 \\
        0 & 0 & 1
    \end{bmatrix} + \begin{bmatrix}
        0 & -1 & 0 \\
        1 & 0 & 0 \\
        0 & 0 & 0
    \end{bmatrix} \sin{\theta_{1}} + \begin{bmatrix}
        0 & -1 & 0 \\
        1 & 0 & 0 \\
        0 & 0 & 0
    \end{bmatrix}^{2} (1 - \cos{\theta_{1}}) \\
    &= \begin{bmatrix}
        \cos{\theta_{1}} & - \sin{\theta_{1}} & 0 \\
        \sin{\theta_{1}} & \cos{\theta_{1}} & 0 \\
        0 & 0 & 1
    \end{bmatrix},
\end{aligned} \\
\text{Similarly,} \qquad
e^{[\hat{\omega}_{1}] \theta_{2}} = \begin{bmatrix}
        \cos{\theta_{2}} & - \sin{\theta_{2}} & 0 \\
        \sin{\theta_{2}} & \cos{\theta_{2}} & 0 \\
        0 & 0 & 1
    \end{bmatrix}. \\
\vec{r}_{1} (0) = \begin{pmatrix}
    0 \\ 0 \\ 0
\end{pmatrix}, \qquad
\vec{r}_{m_{1}} (0) = \begin{pmatrix}
    l_{c1} \\ 0 \\ 0
\end{pmatrix}, \\
\vec{r}_{2} (0) = \begin{pmatrix}
    l_{1} \\ 0 \\ 0
\end{pmatrix}, \qquad
\vec{r}_{m_{2}} (0) = \begin{pmatrix}
    l_{1} + l_{c2} \\ 0 \\ 0
\end{pmatrix}. \\
{}_{S}^{T}\! T_{1} (0) = \begin{bmatrix}
    1 & 0 & 0 & 0 \\
    0 & 1 & 0 & 0 \\
    0 & 0 & 1 & 0 \\
    0 & 0 & 0 & 1
\end{bmatrix}, \qquad
{}_{S}^{T}\! T_{m_{1}} (0) = \begin{bmatrix}
    1 & 0 & 0 & l_{c1} \\
    0 & 1 & 0 & 0 \\
    0 & 0 & 1 & 0 \\
    0 & 0 & 0 & 1
\end{bmatrix}, \\
{}_{S}^{T}\! T_{2} (0) = \begin{bmatrix}
    1 & 0 & 0 & l_{1} \\
    0 & 1 & 0 & 0 \\
    0 & 0 & 1 & 0 \\
    0 & 0 & 0 & 1
\end{bmatrix}, \qquad
{}_{S}^{T}\! T_{m_{2}} (0) = \begin{bmatrix}
    1 & 0 & 0 & l_{1} + l_{c2} \\
    0 & 1 & 0 & 0 \\
    0 & 0 & 1 & 0 \\
    0 & 0 & 0 & 1
\end{bmatrix}. \\
\hat{\xi}_{1} = \begin{pmatrix}
    \hat{\omega}_{1} \\ \vec{v}_{1}
\end{pmatrix} = \begin{pmatrix}
    \hat{\omega}_{1} \\ [\vec{r}_{1}] \hat{\omega}_{1}
\end{pmatrix}
= \begin{pmatrix}
    0 \\ 0 \\ 1 \\ 0 \\ 0 \\ 0
\end{pmatrix}, \\
\hat{\xi}_{2} = \begin{pmatrix}
    \hat{\omega}_{2} \\ \vec{v}_{2}
\end{pmatrix} = \begin{pmatrix}
    \hat{\omega}_{2} \\ [\vec{r}_{2}] \hat{\omega}_{2}
\end{pmatrix}
= \begin{pmatrix}
    0 \\ 0 \\ 1 \\ 0 \\ -l_{1} \\ 0
\end{pmatrix}. \\
\begin{aligned}
    e^{[\hat{\xi}_{1}] \theta_{1}} &= \begin{bmatrix}
        e^{[\hat{\omega}_{1}] \theta_{1}} & (I - e^{[\hat{\omega}_{1}] \theta_{1}}) [\hat{\omega}_{1}] \vec{v}_{1} + \hat{\omega}_{1} \hat{\omega}_{1}^{\top} \vec{v}_{1} \theta_{1} \\
        0 & 1
    \end{bmatrix} \\
    &= \begin{bmatrix}
        \cos{\theta_{1}} & - \sin{\theta_{1}} & 0 & 0 \\
        \sin{\theta_{1}} & \cos{\theta_{1}} & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix},
\end{aligned} \\
\begin{aligned}
    e^{[\hat{\xi}_{2}] \theta_{2}} &= \begin{bmatrix}
        e^{[\hat{\omega}_{2}] \theta_{2}} & (I - e^{[\hat{\omega}_{2}] \theta_{2}}) [\hat{\omega}_{2}] \vec{v}_{2} + \hat{\omega}_{2} \hat{\omega}_{2}^{\top} \vec{v}_{2} \theta_{2} \\
        0 & 1
    \end{bmatrix} \\
    &= \begin{bmatrix}
        \cos{\theta_{2}} & - \sin{\theta_{2}} & 0 & l_{1} (1 - \cos{\theta_{2}}) \\
        \sin{\theta_{2}} & \cos{\theta_{2}} & 0 & - l_{1} \sin{\theta_{2}} \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix}.
\end{aligned} \\
\begin{aligned}
    {}_{S}^{T}\! T_{m_{1}} (\vec{\theta}) &= e^{[\hat{\xi}_{1}] \theta_{1}} {}_{S}^{T}\! T_{m_{1}} (0) \\
    &= \begin{bmatrix}
        \cos{\theta_{1}} & - \sin{\theta_{1}} & 0 & 0 \\
        \sin{\theta_{1}} & \cos{\theta_{1}} & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix} \begin{bmatrix}
        1 & 0 & 0 & l_{c1} \\
        0 & 1 & 0 & 0 \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix} \\
    &= \begin{bmatrix}
        \cos{\theta_{1}} & - \sin{\theta_{1}} & 0 & l_{c1} \cos{\theta_{1}} \\
        \sin{\theta_{1}} & \cos{\theta_{1}} & 0 & l_{c1} \sin{\theta_{1}} \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix},
\end{aligned} \\
\begin{aligned}
    {}_{S}^{T}\! T_{m_{2}} (\vec{\theta}) &= e^{[\hat{\xi}_{1}] \theta_{1}} e^{[\hat{\xi}_{2}] \theta_{2}} {}_{S}^{T}\! T_{m_{2}} (0) \\
    &= \begin{bmatrix}
        \cos{(\theta_{1} + \theta_{2})} & - \sin{(\theta_{1} + \theta_{2})} & 0 & l_{1} \cos{\theta_{1}} + l_{c2} \cos{(\theta_{1} + \theta_{2})} \\
        \sin{(\theta_{1} + \theta_{2})} & \cos{(\theta_{1} + \theta_{2})} & 0 & l_{1} \sin{\theta_{1}} + l_{c2} \sin{(\theta_{1} + \theta_{2})} \\
        0 & 0 & 1 & 0 \\
        0 & 0 & 0 & 1
    \end{bmatrix}.
\end{aligned} \\
\begin{aligned}
    \vec{p}_{m_{1}} (\vec{\theta}) &= {}_{S}^{T}\! T_{m_{1}} (\vec{\theta}) \vec{p}_{0} (\vec{\theta}) \\
    \begin{pmatrix}
        \vec{r}_{m_{1}} (\vec{\theta}) \\ 1
    \end{pmatrix} &= {}_{S}^{T}\! T_{m_{1}} (\vec{\theta}) \begin{pmatrix}
        0 \\ 0 \\ 0 \\ 1
    \end{pmatrix} \\
    \Rightarrow \vec{r}_{m_{1}} (\vec{\theta}) &= \begin{pmatrix}
        l_{c1} \cos{\theta_{1}} \\ l_{c1} \sin{\theta_{1}} \\ 0
    \end{pmatrix}.
\end{aligned} \\
\text{Similarly,} \qquad
\vec{r}_{1} (\vec{\theta}) = \begin{pmatrix}
    l_{1} \cos{\theta_{1}} \\ l_{1} \sin{\theta_{1}} \\ 0
\end{pmatrix}, \qquad
\vec{r}_{m_{2}} (\vec{\theta}) = \begin{pmatrix}
    l_{1} \cos{\theta_{1}} + l_{c2} \cos{(\theta_{1} + \theta_{2})} \\
    l_{1} \sin{\theta_{1}} + l_{c2} \sin{(\theta_{1} + \theta_{2})} \\
    0
\end{pmatrix}. \\
\hat{\omega}_{1}' = e^{[\hat{\xi}_{1}]} \hat{\omega}_{1} = \begin{pmatrix}
    0 \\ 0 \\ 1
\end{pmatrix}, \qquad
\hat{\omega}_{2}' = e^{[\hat{\xi}_{1}]} e^{[\hat{\xi}_{2}]} \hat{\omega}_{2} = \begin{pmatrix}
    0 \\ 0 \\ 1
\end{pmatrix}. \\
\begin{aligned}
    J_{m_{1}} &= \begin{bmatrix}
        \hat{\omega}_{1}' & 0 \\
        \vec{r}_{m_{1}}' \times \hat{\omega}_{1}' & 0
    \end{bmatrix} \\
    &= \begin{bmatrix}
        \hat{\omega}_{1}' & 0 \\
        \vec{r}_{m_{1}} (\vec{\theta}) \times \hat{\omega}_{1}' & 0
    \end{bmatrix} \\
    &= \begin{bmatrix}
        0 & 0 \\
        0 & 0 \\
        1 & 0 \\
        - l_{c1} \sin{\theta_{1}} & 0 \\
        l_{c1} \cos{\theta_{1}} & 0 \\
        0 & 0
    \end{bmatrix},
\end{aligned} \\
\begin{aligned}
    J_{m_{2}} &= \begin{bmatrix}
        \hat{\omega}_{1}' & \hat{\omega}_{2}' \\
        \vec{r}_{m_{2}}' \times \hat{\omega}_{1}' & (\vec{r}_{m_{2}}' - \vec{r}_{m_{1}}') \times \hat{\omega}_{2}'
    \end{bmatrix} \\
    &= \begin{bmatrix}
        \hat{\omega}_{1}' & \hat{\omega}_{2}' \\
        \vec{r}_{m_{2}} (\vec{\theta}) \times \hat{\omega}_{1}' & \left (\vec{r}_{m_{2}} (\vec{\theta}) - \vec{r}_{m_{1}} (\vec{\theta}) \right) \times \hat{\omega}_{2}'
    \end{bmatrix} \\
    &= \begin{bmatrix}
        0 & 0 \\
        0 & 0 \\
        1 & 1 \\
        - l_{1} \sin{\theta_{1}} - l_{c2} \sin{(\theta_{1} + \theta_{2})} & - l_{c2} \sin{(\theta_{1} + \theta_{2})} \\
        l_{1} \cos{\theta_{1}} + l_{c2} \cos(\theta_{1} + \theta_{2}){} & l_{c2} \cos{(\theta_{1} + \theta_{2})} \\
        0 & 0
    \end{bmatrix}.
\end{aligned}
$$

The general mass matrixes $M_{1}$ and $M_{2}$ are

$$
M_{1} = \begin{bmatrix}
    {}^{c} \! \mathcal{I}_{1} & 0 \\
    0 & m_{1} I_{3}
\end{bmatrix} = \begin{bmatrix}
    J_{1x} & 0 & 0 & 0 & 0 & 0 \\
    0 & J_{1y} & 0 & 0 & 0 & 0 \\
    0 & 0 & J_{1} & 0 & 0 & 0 \\
    0 & 0 & 0 & m_{1} & 0 & 0 \\
    0 & 0 & 0 & 0 & m_{1} & 0 \\
    0 & 0 & 0 & 0 & 0 & m_{1}
\end{bmatrix}, \\
M_{2} = \begin{bmatrix}
    {}^{c} \! \mathcal{I}_{2} & 0 \\
    0 & m_{2} I_{3}
\end{bmatrix} = \begin{bmatrix}
    J_{2x} & 0 & 0 & 0 & 0 & 0 \\
    0 & J_{2y} & 0 & 0 & 0 & 0 \\
    0 & 0 & J_{2} & 0 & 0 & 0 \\
    0 & 0 & 0 & m_{2} & 0 & 0 \\
    0 & 0 & 0 & 0 & m_{2} & 0 \\
    0 & 0 & 0 & 0 & 0 & m_{2}
\end{bmatrix}.
$$

Mass Matrix $M(\vec{\theta})$ is

$$
\begin{aligned}
    M (\vec{\theta}) &= \sum_{i = 1}^{2} J_{m_{i}}^{\top} (\vec{\theta}) M_{i} J_{m_{i}} (\vec{\theta}) \\
    &= \begin{bmatrix}
        M_{11} & M_{12} \\
        M_{21} & M_{22}
    \end{bmatrix}.
\end{aligned}
$$

Where,

$$
M_{11} = J_{1} + J_{2} + m_{1} l_{c1}^{2} + m_{2} (l_{1}^{2} + l_{c2}^{2}) + 2 m_{2} l_{1} l_{2} \cos{\theta_{2}}, \\
M_{12} = M_{21} = J_{2} + m_{2} l_{c2}^{2} + m_{2} l_{1} l_{c2} \cos{\theta_{2}}, \qquad
M_{22} = J_{2} + m_{2} l_{c2}^{2}.
$$
