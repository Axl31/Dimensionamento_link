function output2 = controlla_differenza_link (a2,a3,a4)
%FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK  
if ( ((a2-a3)>0) && ((a2-a3)<=4) &&...
        ((a3-a4)>=0) ) %definisco la mia scrematura, voglio che il link 2 sia
    %maggiore del link 3 e che i due non abbiano lunghezze troppo diverse
    check=true;
else
    check=false;
end

output2=check;
end