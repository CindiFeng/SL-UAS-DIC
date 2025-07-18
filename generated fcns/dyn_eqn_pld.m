function out1 = dyn_eqn_pld(x_s1,x_s2,x_s3,x_s4,x_s5,v_s1,v_s2,v_s3,v_s4,v_s5,a_q1,a_q2,a_q3,g,l_p,m_p,m_q)
%DYN_EQN_PLD
%    OUT1 = DYN_EQN_PLD(X_S1,X_S2,X_S3,X_S4,X_S5,V_S1,V_S2,V_S3,V_S4,V_S5,A_Q1,A_Q2,A_Q3,G,L_P,M_P,M_Q)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    03-Sep-2022 18:10:28

t2 = cos(x_s4);
t3 = cos(x_s5);
t4 = sin(x_s4);
t5 = sin(x_s5);
t6 = 1.0./l_p;
out1 = [(t3.*t6.*(a_q1.*t2-a_q3.*t4-g.*t4+l_p.*t5.*v_s4.*v_s5.*2.0))./(t3.^2+1.0);t6.*(a_q2.*t3+a_q1.*t4.*t5+a_q3.*t2.*t5+g.*t2.*t5+l_p.*t3.*t5.*v_s4.^2).*(-1.0./2.0)];
