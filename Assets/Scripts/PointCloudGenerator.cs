using System.IO;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using System;

public class PointCloudGenerator : MonoBehaviour
{
    [Header("Number Of Points In Point Clouds")]
    [SerializeField] public int numberOfPoints1 = 6; // The number of points generated in pointCloud1.txt file
    [SerializeField] public int numberOfPoints2 = 6; // The number of points generated in pointCloud2.txt file

    [Header("Random Point Generation Range")]
    public float minRange = 0f;    // Min range of values of coordinates of points
    public float maxRange = 15f;   // Max range of values of coordinates of points

    [Header("Cube Example")]
    public bool isDesiredCubeExample = false;   // Check if the user wants to see the cube example


    public void CreatePointClouds()
    {
        string path1 = Path.Combine(Application.dataPath, "pointCloud1.txt");
        string path2 = Path.Combine(Application.dataPath, "pointCloud2.txt");

        int commonNumber = 0;

        if (numberOfPoints1 < numberOfPoints2) commonNumber = Mathf.CeilToInt(numberOfPoints1 / 2f);
        else commonNumber = Mathf.CeilToInt(numberOfPoints2 / 2f);


        // Generate random points for each set of points
        Vector3[] commonPoints = GenerateRandomPoints(commonNumber);  // At least half of them are exact matches
        Vector3[] uniquePoints1 = GenerateRandomPoints(numberOfPoints1 - commonNumber);
        Vector3[] uniquePoints2 = GenerateRandomPoints(numberOfPoints2 - commonNumber);

        // Combine to create two sets
        Vector3[] points1 = CombinePoints(commonPoints, uniquePoints1);
        Vector3[] points2 = CombinePoints(commonPoints, uniquePoints2);

        // Shuffle points
        ShufflePoints(points1);
        ShufflePoints(points2);

        // Write to files
        WritePointsToFile(path1, points1);
        WritePointsToFile(path2, points2);

        // Read points back from files
        List<Vector3> P = ReadPointsFromFile(path1);
        List<Vector3> Q = ReadPointsFromFile(path2);

        CleanAllCloudsAndLines();

        // Create spheres for visualization
        CreateSpheres(P, Color.blue, "PointCloud1");  // P point cloud - Blue
        CreateSpheres(Q, new Color(1.0f, 0.5f, 0.0f), "PointCloud2"); // Q point cloud - Orange
    }

    public void LoadPointClouds()
    {

        // Define the file paths
        string path1 = Path.Combine(Application.dataPath, "pointCloud1.txt");
        string path2 = Path.Combine(Application.dataPath, "pointCloud2.txt");

        string cubePath1 = Path.Combine(Application.dataPath, "CubeCloud1.txt");
        string cubePath2 = Path.Combine(Application.dataPath, "CubeCloud2.txt");

        // Check the isDesiredCubeExample condition
        if (isDesiredCubeExample)
        {
            try
            {
                // Copy the contents of CubeCloud1.txt to pointCloud1.txt
                if (File.Exists(cubePath1))
                {
                    File.Copy(cubePath1, path1, overwrite: true);
                }
                else
                {
                    Debug.LogError($"File not found: {cubePath1}");
                }

                // Copy the contents of CubeCloud2.txt to pointCloud2.txt
                if (File.Exists(cubePath2))
                {
                    File.Copy(cubePath2, path2, overwrite: true);
                }
                else
                {
                    Debug.LogError($"File not found: {cubePath2}");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error while copying files: {ex.Message}");
            }
        }


        // Read points back from files
        List<Vector3> P = ReadPointsFromFile(path1);
        List<Vector3> Q = ReadPointsFromFile(path2);

        CleanAllCloudsAndLines();

        // Create spheres for visualization
        CreateSpheres(P, Color.blue, "PointCloud1"); // P - Blue
        CreateSpheres(Q, new Color(1.0f, 0.5f, 0.0f), "PointCloud2"); // Q - Orange
    }

   
    Vector3[] GenerateRandomPoints(int count)
    {
        Vector3[] points = new Vector3[count];
        for (int i = 0; i < count; i++)
        {
            points[i] = new Vector3(
                UnityEngine.Random.Range(minRange, maxRange),
                UnityEngine.Random.Range(minRange, maxRange),
                UnityEngine.Random.Range(minRange, maxRange)
            );
        }
        return points;
    }

    Vector3[] CombinePoints(Vector3[] commonPoints, Vector3[] uniquePoints)
    {
        Vector3[] combined = new Vector3[commonPoints.Length + uniquePoints.Length];
        commonPoints.CopyTo(combined, 0);
        uniquePoints.CopyTo(combined, commonPoints.Length);
        return combined;
    }

    void ShufflePoints(Vector3[] points)
    {
        for (int i = 0; i < points.Length; i++)
        {
            int randomIndex = UnityEngine.Random.Range(0, points.Length);
            Vector3 temp = points[i];
            points[i] = points[randomIndex];
            points[randomIndex] = temp;
        }
    }

    void WritePointsToFile(string path, Vector3[] points)
    {
        using (StreamWriter writer = new StreamWriter(path))
        {
            writer.WriteLine(points.Length);
            foreach (var point in points)
            {
                writer.WriteLine($"{point.x:F4} {point.y:F4} {point.z:F4}");
            }
        }
    }

    List<Vector3> ReadPointsFromFile(string path)
    {
        List<Vector3> points = new List<Vector3>();

        using (StreamReader reader = new StreamReader(path))
        {
            string line = reader.ReadLine(); // First line is the number of points
            while ((line = reader.ReadLine()) != null)
            {
                string[] parts = line.Split(' ');
                if (parts.Length == 3)
                {
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    points.Add(new Vector3(x, y, z));
                }
            }
        }

        return points;
    }

    void CreateSpheres(List<Vector3> points, Color color, string parentName)
    {
        // Create a parent object
        GameObject parentObject = new GameObject(parentName);
        int i = 1;

        foreach (var point in points)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = point;
            sphere.transform.localScale = Vector3.one * 0.5f; // Adjust size of spheres

            Renderer renderer = sphere.GetComponent<Renderer>();
            renderer.material.color = color;

            // Set the name of the sphere to "Sphere"
            sphere.name = "Sphere" + i.ToString();

            i++;

            // Set the parent of the sphere to the created parent object
            sphere.transform.SetParent(parentObject.transform);
        }
    }


    // Function to clean all game objects with the name "Sphere"
    public void CleanAllCloudsAndLines()
    {
        // Find the parent object by its name
        GameObject parentObject = GameObject.Find("PointCloud1");

        if (parentObject == null)
        {
            return;
        }

        // Destroy the parent object along with all its children
        Destroy(parentObject);

        // Find the parent object by its name
        GameObject parentObject2 = GameObject.Find("PointCloud2");

        if (parentObject2 == null)
        {
            return;
        }

        // Destroy the parent object along with all its children
        Destroy(parentObject2);

        // Find the parent object by its name
        GameObject parentObject3 = GameObject.Find("TransformedPointCloud");

        if (parentObject3 == null)
        {
            return;
        }

        // Destroy the parent object along with all its children
        Destroy(parentObject3);

        // Find the parent object by its name
        GameObject parentObject4 = GameObject.Find("PointCloudConnections");

        if (parentObject4 == null)
        {
            return;
        }

        // Destroy the parent object along with all its children
        Destroy(parentObject4);
    }


}




