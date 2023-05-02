function R = RotationMatrix(Euler)

    Phi   = Euler(1);
    Theta = Euler(2);
    Psi   = Euler(3);

    cPhi = cos(Phi);
    sPhi = sin(Phi);
    cThe = cos(Theta);
    sThe = sin(Theta);
    cPsi = cos(Psi);
    sPsi = sin(Psi);

    R = [cThe * cPsi , sPhi * sThe * cPsi - cPhi * sPsi , cPhi * sThe * cPsi + sPhi * sPsi
         cThe * sPsi , sPhi * sThe * sPsi + cPhi * cPsi , cPhi * sThe * sPsi - sPhi * cPsi
         -sThe       , cThe * sPhi                      , cThe * cPhi];
end