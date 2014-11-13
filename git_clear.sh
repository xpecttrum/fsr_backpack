#!/bin/bash
#
# Script made by: Jose Prado
#
Muda_Dir_e_Filhos(){

 n=$(expr $n + 1)


 i[$n]=1
 numlinhas[$n]=$(ls -l | grep ^d | cut -d\: -f2 | cut -d\  -f2 | wc -l)

#echo "numero de subdiretorios(nivel="$n") = "${numlinhas[$n]}

#entra em todos os subdiretorios 1 nivel soh
while [ ${i[$n]} -le ${numlinhas[$n]} ]; do

   dirname[$n]=$(ls -l | grep ^d | cut -d\: -f2 | cut -d\  -f2,3,4,5,6,7,8,9 | head -n${i[$n]} | tail -n1)
   
   rm -rf "${dirname[$n]}/.git"
   echo $n")- "${dirname[$n]}"- GIT reference cleared OK"
     

   cd "${dirname[$n]}"
   Muda_Dir_e_Filhos
   n=$(expr $n - 1)
   
   cd ..	

    i[$n]=$(expr ${i[$n]} + 1)
done

}

rm -rf .svn
n=0
Muda_Dir_e_Filhos

echo "GIT clear done - Script by: Jose Prado"

