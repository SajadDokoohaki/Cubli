H_new = ss(A,B,C,D);


X0 = [pi/3, pi/3, pi/3, pi/3, pi/3];
x0 = diag(X0);

N = 3000;
Tf = 1/N;
D = -0.5;

figure
for i = 1:5

    subplot(2,3,i)

    for P = linspace(-1,-0.1,5)
    
        for I = linspace(-1,-0.1,10)
    
                G_c = pid(P, I, D, Tf);
        
                sys_mul = feedback(G_c * H_new, 1) ;
        
                isproper(sys_mul)
        
                initial(sys_mul, x0(:, i))
        
                hold on;
    
        end
    
    end

end


% [t,s] = title({'Closed Loop',[num2str(-1),'$<P<$',num2str(0),'****' ...,
%     ,num2str(-10),'$<I<$',num2str(-1),'****', 'D = ', num2str(D) ...
%     ,'tf = ',num2str(tf)]}, ...
%     'Color', 'Blue', 'Interpreter', 'latex');
