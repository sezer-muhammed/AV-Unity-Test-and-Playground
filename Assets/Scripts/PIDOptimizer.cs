using UnityEngine;
using System.Collections.Generic;
using System.IO;
using System.Linq;

public class PIDOptimizer : MonoBehaviour
{
    [Header("Genetic Algorithm Settings")]
    public GameObject carPrefab;
    public int populationSize = 10;
    public int generations = 5;
    public float generationDuration = 15f;
    public Vector3 targetPosition = new Vector3(0, 0, 80);

    [Header("Spawn / Collision")]
    public bool spawnAllAtSamePoint = true;
    public Vector3 spawnPosition = new Vector3(0, 1, 0);
    public Vector3 spawnEulerAngles = Vector3.zero;
    
    [Header("Parameter Ranges")]
    public float minSensitivity = 0.5f, maxSensitivity = 4.0f;
    public float minLookAhead = 3.0f, maxLookAhead = 10.0f;
    public float mutationRate = 0.15f;

    private int currentGeneration = 0;
    private float timer = 0f;
    private bool isTesting = false;
    private List<CarAgent> population = new List<CarAgent>();

    [System.Serializable]
    public class Genome
    {
        public float sensitivity;
        public float lookAhead;
        public float fitness = 0;

        public Genome(float s, float l) { sensitivity = s; lookAhead = l; }

        public Genome Clone()
        {
            return new Genome(sensitivity, lookAhead) { fitness = fitness };
        }
    }

    private class CarAgent
    {
        public GameObject instance;
        public PurePursuitController controller;
        public Genome genome;
        public float minDistanceToTarget = float.MaxValue;
        public bool hasArrived = false;
        public float arrivalTime = -1f;

        public float CalculateFitness(float maxTime)
        {
            float distScore = 1f / (1f + minDistanceToTarget); 
            float timeScore = hasArrived ? (maxTime - arrivalTime + 10f) : 0f;
            return distScore * 10f + timeScore;
        }
    }

    private List<Genome> currentGenomes = new List<Genome>();

    void Start()
    {
        if (carPrefab == null) { Debug.LogError("Assign Car Prefab!"); return; }
        
        // Initial random population
        for (int i = 0; i < populationSize; i++)
        {
            currentGenomes.Add(new Genome(
                Random.Range(minSensitivity, maxSensitivity),
                Random.Range(minLookAhead, maxLookAhead)
            ));
        }

        StartGeneration();
    }

    void StartGeneration()
    {
        timer = 0;
        isTesting = true;
        population.Clear();

        Debug.Log($"<color=cyan>Starting Generation {currentGeneration + 1}/{generations}</color>");

        for (int i = 0; i < populationSize; i++)
        {
            Vector3 pos = spawnAllAtSamePoint ? spawnPosition : (spawnPosition + new Vector3(i * 3f, 0, 0));
            Quaternion rot = Quaternion.Euler(spawnEulerAngles);
            GameObject car = Instantiate(carPrefab, pos, rot);
            car.name = $"Gen{currentGeneration}_Car{i}";

            // Ignore collisions between this car and already spawned cars (keep ground collisions ON).
            IgnoreCollisionsWithExistingCars(car);

            PurePursuitController controller = car.GetComponent<PurePursuitController>();
            if (controller != null)
            {
                controller.steeringSensitivity = currentGenomes[i].sensitivity;
                controller.lookAheadDistance = currentGenomes[i].lookAhead;
                controller.isAutonomous = true;
                controller.SetForceTarget(targetPosition);
                
                // Ensure car actually moves: kickstart acceleration
                var pcc = car.GetComponent<PrometeoCarController>();
                if (pcc != null) {
                    pcc.useExternalInput = true;
                }

                population.Add(new CarAgent {
                    instance = car,
                    controller = controller,
                    genome = currentGenomes[i]
                });
            }
        }
    }

    void IgnoreCollisionsWithExistingCars(GameObject newCar)
    {
        Collider[] newColliders = newCar.GetComponentsInChildren<Collider>(true);
        if (newColliders == null || newColliders.Length == 0) return;

        foreach (var existing in population)
        {
            if (existing == null || existing.instance == null) continue;

            Collider[] existingColliders = existing.instance.GetComponentsInChildren<Collider>(true);
            if (existingColliders == null || existingColliders.Length == 0) continue;

            foreach (var a in newColliders)
            {
                if (a == null) continue;
                foreach (var b in existingColliders)
                {
                    if (b == null) continue;
                    Physics.IgnoreCollision(a, b, true);
                }
            }
        }
    }

    void Update()
    {
        if (!isTesting) return;

        timer += Time.deltaTime;

        foreach (var agent in population)
        {
            if (agent.instance == null) continue;

            float d = Vector3.Distance(agent.instance.transform.position, targetPosition);
            if (d < agent.minDistanceToTarget) agent.minDistanceToTarget = d;

            if (!agent.hasArrived && d < 4.0f)
            {
                agent.hasArrived = true;
                agent.arrivalTime = timer;
            }
        }

        if (timer >= generationDuration)
        {
            EndGeneration();
        }
    }

    void EndGeneration()
    {
        isTesting = false;

        // Calculate fitness
        foreach (var agent in population)
        {
            agent.genome.fitness = agent.CalculateFitness(generationDuration);
            Destroy(agent.instance);
        }

        AppendGenerationCsv();

        currentGeneration++;
        if (currentGeneration >= generations)
        {
            var best = currentGenomes.OrderByDescending(g => g.fitness).First();
            Debug.Log($"<color=green>Evolution Complete! BEST: Sens={best.sensitivity:F2}, Look={best.lookAhead:F2}</color>");
            return;
        }

        // Selection & Crossover
        List<Genome> nextGen = new List<Genome>();
        var sorted = currentGenomes.OrderByDescending(g => g.fitness).ToList();
        
        // Elite: keep top 2
        nextGen.Add(sorted[0].Clone());
        nextGen.Add(sorted[1].Clone());

        while (nextGen.Count < populationSize)
        {
            // Roulette selection for parents
            Genome p1 = SelectParent(sorted);
            Genome p2 = SelectParent(sorted);
            
            // Crossover
            Genome child = new Genome(
                Random.value > 0.5f ? p1.sensitivity : p2.sensitivity,
                Random.value > 0.5f ? p1.lookAhead : p2.lookAhead
            );

            // Mutation
            if (Random.value < mutationRate) child.sensitivity += Random.Range(-0.5f, 0.5f);
            if (Random.value < mutationRate) child.lookAhead += Random.Range(-1f, 1f);
            
            child.sensitivity = Mathf.Clamp(child.sensitivity, minSensitivity, maxSensitivity);
            child.lookAhead = Mathf.Clamp(child.lookAhead, minLookAhead, maxLookAhead);

            nextGen.Add(child);
        }

        currentGenomes = nextGen;
        StartGeneration();
    }

    void AppendGenerationCsv()
    {
        try
        {
            string path = Path.Combine(Application.persistentDataPath, "GA_Optimization_Results.csv");
            bool writeHeader = !File.Exists(path);

            using (var writer = new StreamWriter(path, append: true))
            {
                if (writeHeader)
                {
                    writer.WriteLine("Generation,Index,Sensitivity,LookAhead,Fitness");
                }

                var sorted = currentGenomes.OrderByDescending(g => g.fitness).ToList();
                for (int i = 0; i < sorted.Count; i++)
                {
                    var g = sorted[i];
                    writer.WriteLine($"{currentGeneration + 1},{i},{g.sensitivity:F4},{g.lookAhead:F4},{g.fitness:F6}");
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Failed to write GA results CSV: " + e.Message);
        }
    }

    Genome SelectParent(List<Genome> sorted)
    {
        float total = sorted.Sum(g => g.fitness);
        float r = Random.Range(0, total);
        float count = 0;
        foreach (var g in sorted)
        {
            count += g.fitness;
            if (count >= r) return g;
        }
        return sorted[0];
    }
}
