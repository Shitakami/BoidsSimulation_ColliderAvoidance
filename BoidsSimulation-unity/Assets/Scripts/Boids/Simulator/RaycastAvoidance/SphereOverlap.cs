using System.Collections.Generic;
using UnityEngine;

public class SphereOverlap : MonoBehaviour
{
    public int gridSize = 3;
    public float N = 2f;
    public int iterations = 100;

    [ContextMenu("Calculate")]
    void Start()
    {
        List<int> overlappedGridsList = new List<int>();

        for (int i = 0; i < iterations; i++)
        {
            Vector3 sphereCenter = new Vector3(
                Random.Range(-N / 2, N / 2),
                Random.Range(-N / 2, N / 2),
                Random.Range(-N / 2, N / 2)
            );

            int overlappedGrids = CalculateOverlappingGrids(sphereCenter);
            overlappedGridsList.Add(overlappedGrids);
        }

        float average = CalculateAverage(overlappedGridsList);
        float median = CalculateMedian(overlappedGridsList);

        Debug.Log("Average Overlapped Grids: " + average);
        Debug.Log("Median Overlapped Grids: " + median);
    }

    int CalculateOverlappingGrids(Vector3 sphereCenter)
    {
        int overlappedGrids = 0;
        float cellSize = N / 2;

        for (int x = 0; x < gridSize; x++)
        {
            for (int y = 0; y < gridSize; y++)
            {
                for (int z = 0; z < gridSize; z++)
                {
                    Vector3 cellMin = new Vector3(
                        -N / 2 + x * cellSize,
                        -N / 2 + y * cellSize,
                        -N / 2 + z * cellSize
                    );
                    Vector3 cellMax = cellMin + new Vector3(cellSize, cellSize, cellSize);

                    Vector3 closestPoint = new Vector3(
                        Mathf.Clamp(sphereCenter.x, cellMin.x, cellMax.x),
                        Mathf.Clamp(sphereCenter.y, cellMin.y, cellMax.y),
                        Mathf.Clamp(sphereCenter.z, cellMin.z, cellMax.z)
                    );

                    float distance = Vector3.Distance(closestPoint, sphereCenter);
                    if (distance <= N)
                    {
                        overlappedGrids++;
                    }
                }
            }
        }

        return overlappedGrids;
    }

    float CalculateAverage(List<int> data)
    {
        float sum = 0;
        foreach (int value in data)
        {
            sum += value;
        }
        return sum / data.Count;
    }

    float CalculateMedian(List<int> data)
    {
        data.Sort();

        if (data.Count % 2 == 0)
        {
            return (data[data.Count / 2 - 1] + data[data.Count / 2]) / 2f;
        }
        else
        {
            return data[data.Count / 2];
        }
    }
}
