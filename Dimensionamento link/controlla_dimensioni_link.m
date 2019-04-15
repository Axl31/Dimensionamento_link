function output = controlla_dimensioni_link(p, theta, q4, a,lim)
%FUNZIONE UTILIZZATA PER IL MAIN PER OTTIMIZZARE I LINK  
    %da in uscita un valore booleano e fa il check il controllo di quanto
    %dato in ingresso

    output=true;

    if(~isempty(p) && ~isempty(theta))
        %& metto i primi due valori del vettore che ho messo
        p_=p(1,:);
        theta_=theta(1);
        %& li scorro per vedere la lunghezza e poi vedo se questi valori sono
        %compatibili con quelli del nostro workspace
        if length(p)>1 && length(theta)>1%%passo all'elemento successivo
            p = p(2:end,:); %& è come se eliminassi il primo elemento?
            theta = theta(2:end);
        else 
            p = [];
            theta = [];
        end

        %& blocco di inversione analitica per passare dallo spazio operativo a
        %quello dei giunti. In Q ho la configurazione degli angoli di giunto
        %che ci aspettiamo


        Q=cinematica_inversa_4gdl(p_, theta_, a, q4);

        %& verifica se Q è un vettore vuoto e se appartine e a quell'intervallo
        %impostato inizialmente
        if (~isempty(Q) && Q(1)>lim(1,1) && Q(1)<lim(1,2) ...
                          && Q(2)>lim(2,1) && Q(2)<lim(2,2) ...
                          && Q(3)>lim(3,1) && Q(3)<lim(3,2) ...
                          && Q(4)>lim(4,1) && Q(4)<lim(4,2))
           
            output = output && controlla_dimensioni_link(p, theta, q4, a, lim);
            %& è una funzione di tipo ricorsivo 
            %& poichè lo devo fare per tutte le p e theta e vedo se è
            %accetabile attraverso la scrittura output && check.. Se la
            %funzione mi restituisce falso l'and non funziona più 
        else
            output=false;%se la condizione non è verificata,l'output della funzione sarà false
        end
    end
end