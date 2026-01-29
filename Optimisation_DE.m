% Paramètres du DE
D = 9; % 3 gains x 3 PID = 9 variables à optimiser
NP = 20; % Taille de la population (20 robots virtuels)
F = 0.8; % Facteur d'échelle
CR = 0.9; % Probabilité de croisement
MaxGen = 50; % Nombre d'itérations

% Limites des gains [Kp1 Ki1 Kd1 Kp2 Ki2 Kd2 Kp3 Ki3 Kd3]
L_bound = [0 0 0 0 0 0 0 0 0]; 
U_bound = [100 50 10 100 50 10 100 50 10];

% 1. Initialisation de la population
pop = L_bound + (U_bound - L_bound) .* rand(NP, D);
fitness = zeros(NP, 1);

for i = 1:NP
    fitness(i) = evaluate_pid(pop(i,:)); % On simule chaque PID
end

% 2. Boucle principale du DE
for gen = 1:MaxGen
    for i = 1:NP
        % Mutation et Croisement
        idxs = randperm(NP, 3);
        v = pop(idxs(1),:) + F * (pop(idxs(2),:) - pop(idxs(3),:));
        v = max(min(v, U_bound), L_bound); % Rester dans les limites
        
        j_rand = randi(D);
        u = pop(i,:);
        for j = 1:D
            if rand < CR || j == j_rand
                u(j) = v(j);
            end
        end
        
        % Sélection
        new_fitness = evaluate_pid(u);
        if new_fitness < fitness(i)
            pop(i,:) = u;
            fitness(i) = new_fitness;
        end
    end
    fprintf('Génération %d: Meilleur Score = %f\n', gen, min(fitness));
end

% --- Fonction pour lancer Simulink ---
function score = evaluate_pid(gains)
    % On envoie les gains vers Simulink
    assignin('base', 'Kp1', gains(1)); assignin('base', 'Ki1', gains(2)); assignin('base', 'Kd1', gains(3));
    assignin('base', 'Kp2', gains(4)); assignin('base', 'Ki2', gains(5)); assignin('base', 'Kd2', gains(6));
    assignin('base', 'Kp3', gains(7)); assignin('base', 'Ki3', gains(8)); assignin('base', 'Kd3', gains(9));
    
    % On lance la simulation
    simOut = sim('NOM_DE_VOTRE_MODELE', 'SimulationMode', 'normal');
    
    % On récupère la valeur de l'erreur (ITAE) à la fin
    score = simOut.Performance.Data(end);
end