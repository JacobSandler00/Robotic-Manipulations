syms IzzA IzzB ma mb TA TB qb qa qaD qbD LA LB g qaDD qbDD

qaDD = 2*((4*IzzB+LB*mb*(LB+2*LA*cos(qb)))*(2*TB-g*LB*mb*cos(qa+qb)-LA*LB*mb*sin(qb)*qaD^2)-(4*IzzB+mb*LB^2)*(2*TA-g*LA*ma*cos(qa)-g*mb*(2*LA*cos(qa)+LB*cos(qa+qb))-LA*LB*mb*sin(qb)*(qaD^2-(qaD+qbD)^2)))/((4*IzzB+LB*mb*(LB+2*LA*cos(qb)))^2-(4*IzzB+mb*LB^2)*(4*IzzA+4*IzzB+ma*LA^2+mb*(LB^2+4*LA^2+4*LA*LB*cos(qb))))
qbDD =-2*((4*IzzA+4*IzzB+ma*LA^2+mb*(LB^2+4*LA^2+4*LA*LB*cos(qb)))*(2*TB-g*LB*mb*cos(qa+qb)-LA*LB*mb*sin(qb)*qaD^2)-(4*IzzB+LB*mb*(LB+2*LA*cos(qb)))*(2*TA-g*LA*ma*cos(qa)-g*mb*(2*LA*cos(qa)+LB*cos(qa+qb))-LA*LB*mb*sin(qb)*(qaD^2-(qaD+qbD)^2)))/((4*IzzB+LB*mb*(LB+2*LA*cos(qb)))^2-(4*IzzB+mb*LB^2)*(4*IzzA+4*IzzB+ma*LA^2+mb*(LB^2+4*LA^2+4*LA*LB*cos(qb))))

qaLeft = TA - 0.5*g*LA*ma*cos(qa) - 0.5*g*mb*(2*LA*cos(qa)+LB*cos(qa+qb))
qaRight = IzzA*qaDD + IzzB*(qaDD+qbDD) + 0.25*ma*LA^2*qaDD - 0.25*mb*(2*LA*LB*sin(qb)*(qaD+qbD)^2-2*LA*LB*sin(qb)*qaD^2-4*LA^2*qaDD-LB^2*(qaDD+qbDD)-4*LA*LB*cos(qb)*qaDD-2*LA*LB*cos(qb)*qbDD)

qbLeft = TB - 0.5*g*LB*mb*cos(qa+qb)
qbRight = IzzB*(qaDD+qbDD) + 0.25*LB*mb*(2*LA*sin(qb)*qaD^2+LB*(qaDD+qbDD)+2*LA*cos(qb)*qaDD)


EQN = [qaLeft == qaRight; qbLeft == qbRight]
vars = [qaDD ;qbDD]
solve(EQN, vars)

