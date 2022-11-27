function [Thetab_dd , Thetaw_dd] = fcn(Thetab , Thetab_d , Thetaw_d , u)
   Thetab_dd = ((mb*lb + mw*l)*g*sin(Thetab) - Km*u - Cb*Thetab_d + Cw*Thetaw_d ) / (Ib + mw*l^2) ; 
   Thetaw_dd = ( (Ib + Iw + mw*l^2)*(Km*u - Cw*Thetaw_d))/(Iw *(Ib + mw*l^2))...
               - ((mb*lb + mw*l)*g*sin(Thetab) - Cb*Thetab_d)/(Ib + mw*l^2) ;
end
   

