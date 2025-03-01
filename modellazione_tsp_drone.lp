% REGOLE DI GENERAZIONE DEL CICLO
% Per ogni nodo, deve essere selezionato esattamente un arco uscente
{ cycle(X,Y) : edge(X,Y) } = 1 :- node(X).
% Per ogni nodo, deve essere selezionato esattamente un arco entrante
{ cycle(X,Y) : edge(X,Y) } = 1 :- node(Y).

% REGOLE DI RAGGIUNGIBILITÀ
% Un nodo è raggiungibile se è direttamente collegato al nodo 1
reached(Y) :- cycle(1,Y).
% Un nodo è raggiungibile se è collegato a un nodo già raggiungibile
reached(Y) :- cycle(X,Y), reached(X).
% Non sono ammesse soluzioni con nodi non raggiungibili
:- node(Y), not reached(Y).

% VISUALIZZAZIONE
#show cycle/2.
#show total_distance/1.
#show total_battery/1.

% OTTIMIZZAZIONE
% Minimizza il costo totale del percorso
#minimize { C,X,Y : cycle(X,Y), cost(X,Y,C) }.

% DEFINIZIONE NODI E ARCHI
% Definizione dei nodi (città)
node(1..7).

% Definizione degli archi (collegamenti tra città)
edge(1,(2;3;4)). % Collegamenti da Matera
edge(2,(1;4;5)). % Collegamenti da Montescaglioso
edge(3,(1;4;6)). % Collegamenti da Pomarico
edge(4,(1;2;3;5)). % Collegamenti da Miglionico
edge(5,(2;4;6;7)). % Collegamenti da Grottole
edge(6,(3;5;7)). % Collegamenti da Grassano
edge(7,(5;6)). % Collegamenti da Salandra

% DEFINIZIONE DISTANZE E COSTI BATTERIA
% Distanze in km
distance(1,2,15). distance(2,1,15).
distance(1,3,20). distance(3,1,20).
distance(1,4,12). distance(4,1,12).
distance(2,4,10). distance(4,2,10).
distance(2,5,25). distance(5,2,25).
distance(3,4,15). distance(4,3,15).
distance(3,6,20). distance(6,3,20).
distance(4,5,18). distance(5,4,18).
distance(5,6,15). distance(6,5,15).
distance(5,7,12). distance(7,5,12).
distance(6,7,10). distance(7,6,10).

% Costi batteria in percentuale
battery_cost(1,2,12). battery_cost(2,1,14).
battery_cost(1,3,15). battery_cost(3,1,16).
battery_cost(1,4,10). battery_cost(4,1,11).
battery_cost(2,4,8). battery_cost(4,2,8).
battery_cost(2,5,16). battery_cost(5,2,17).
battery_cost(3,4,11). battery_cost(4,3,11).
battery_cost(3,6,14). battery_cost(6,3,15).
battery_cost(4,5,13). battery_cost(5,4,13).
battery_cost(5,6,12). battery_cost(6,5,12).
battery_cost(5,7,9). battery_cost(7,5,9).
battery_cost(6,7,8). battery_cost(7,6,8).

% CALCOLO COSTI E TOTALI
% Calcolo del costo totale come somma di distanza e consumo batteria
cost(X,Y,C) :- distance(X,Y,D), battery_cost(X,Y,B), C = D + B.

% Calcolo della distanza totale
total_distance(TD) :- TD = #sum { D,X,Y : cycle(X,Y), distance(X,Y,D) }.

% Calcolo del consumo totale di batteria
total_battery(TB) :- TB = #sum { B,X,Y : cycle(X,Y), battery_cost(X,Y,B) }.

% Verifica completezza del ciclo
cycle_complete :- N = #count { X : node(X) }, 
                 N = #count { X : reached(X) }.
:- not cycle_complete.