function M = matric_homogene(X)

M = [cos(X(4)) ,-sin(X(4))*cos(X(2))  ,sin(X(4))*sin(X(2))    ,X(1)*cos(X(4)) ; 
     sin(X(4)) ,cos(X(4))*cos(X(2))  ,-cos(X(4))*sin(X(2))   , X(1)*sin(X(4)) ; 
     0         ,sin(X(2))             ,cos(X(2))               ,X(3) ;
     0        , 0                    , 0                       ,1]   ;

end