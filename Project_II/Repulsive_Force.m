function [ RepFor ] = Repulsive_Force(nu, Oi, b)

distbO(1,1) = b(1,1) - Oi(1,1);
distbO(2,1) = b(2,1) - Oi(2,1);
magnitude = sqrt(distbO(1,1)^2+distbO(2,1)^2);
Force = nu*(1/(magnitude)-1/1)*1/magnitude^2*(Oi-b)/abs(Oi-b);
RepFor(1,1) = Force(1,2);
RepFor(2,1) = Force(2,2);

end
