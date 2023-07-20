H_new = ss(A,B,C,D);

x0 = zeros(5,1) ;
x0(1) = pi/10;

N = 3000;
Tf = 1/N;
D = -0.0295;

figure
for P = linspace(-1,-0.1,5)

    for I = linspace(-10,-1,5)
 
           
            G_c = pid(P, I, D, Tf);
    
            sys_mul = feedback(G_c, H_new) ;
            
            isproper(sys_mul)
    
            initial(sys_mul, x0)
    
            hold on;
    

    end

end

 
[t,s] = title({'Closed Loop',[num2str(-1),'$<P<$',num2str(0),'****' ...,
            ,num2str(-10),'$<I<$',num2str(-1),'****', 'D = ', num2str(D) ...
            ,'tf = ',num2str(tf)]}, ...
            'Color', 'Blue', 'Interpreter', 'latex');
        