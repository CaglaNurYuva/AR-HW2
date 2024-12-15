using System.Collections.Generic;
using System.IO;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra.Double;
using UnityEngine.UI;
using TMPro;
using UnityEditor.PackageManager;

public class RansacAlgorithm : MonoBehaviour
{

    [SerializeField]
    public Text TranslationText;  // Translation matrix on the screen
    public Text RotationText;     // Rotation matrix on the screen

    [Header("RANSAC Parameters")]
    public int theNumberOfIterations = 10000;  // The number of iterations for RANSAC algorithm
    public float threshold = 0.4f;             // Threshold to decide which point is inlier and which point is outlier

    public void ApplyRANSAC()
    {
        int theNumberOfInliers = 0;
        float minError = float.MaxValue;
        Matrix3x3 R = new Matrix3x3();
        Vector3 T = new Vector3(0, 0, 0);
        List<Vector3> transformedQ = new List<Vector3>();


        for (int i = 0; i < theNumberOfIterations; i++) {
            (float error, int theNumberOfInliersTemp, Matrix3x3 R_temp, Vector3 T_temp, List<Vector3> transformedQ_temp) = AlignPointClouds();

            // If threshold is not chosen wisely by the user, this can be exploited and can lead to bad R and T results
            //if (error < minError || theNumberOfInliersTemp > theNumberOfInliers) {
            //    minError = error;
            //    theNumberOfInliers = theNumberOfInliersTemp;
            //    R = R_temp;
            //    T = T_temp;
            //    transformedQ = transformedQ_temp;
            //}

            if (error < minError)
            {
                minError = error;
                theNumberOfInliers = theNumberOfInliersTemp;
                R = R_temp;
                T = T_temp;
                transformedQ = transformedQ_temp;
            }
        }

        // Clean transformed point cloud if any
        CleanTransformedCloudAndLine();

        // Create transformed point cloud
        CreateSpheres(transformedQ, Color.green, "TransformedPointCloud");

        UpdateTranslationText(T);
        UpdateRotationText(R);

        

        Debug.Log($"The Number of Inliers: {theNumberOfInliers}    The Calculated Error: {minError:F4}    R Matrix Determinant: {R.Determinant():F2}");

    }

    public (float leastSquaresError, int theNumberOfInliers, Matrix3x3 R, Vector3 T, List<Vector3> transformedQ) AlignPointClouds()
    {
        string path1 = Path.Combine(Application.dataPath, "pointCloud1.txt");
        string path2 = Path.Combine(Application.dataPath, "pointCloud2.txt");

        List<Vector3> P = ReadPointsFromFile(path1);
        List<Vector3> Q = ReadPointsFromFile(path2);

        if (P.Count < 3 || Q.Count < 3)
        {
            Debug.LogError("Not enough points to perform alignment. Each point cloud must have at least 3 points.");
            return (-1f, -1, null, Vector3.zero, null);
        }

        // Step 1: Randomly select 3 points from P and Q
        List<Vector3> selectedP = SelectRandomPoints(P, 3);
        List<Vector3> selectedQ = SelectRandomPoints(Q, 3);

        // Step 2: Compute centroids Cp and Cq
        Vector3 Cp = ComputeCentroid(selectedP);
        Vector3 Cq = ComputeCentroid(selectedQ);

        // Step 3: Center the points
        List<Vector3> centeredP = CenterPoints(selectedP, Cp);
        List<Vector3> centeredQ = CenterPoints(selectedQ, Cq);

        // Step 4: Construct H matrix
        Matrix3x3 H = ConstructHMatrix(centeredP, centeredQ);

        // Step 5: Perform SVD to find R
        Matrix3x3 R = PerformSVD(H);

        // Step 6: Compute translation vector T
        Vector3 RCp = R.Multiply(Cp);
        Vector3 T = Cq - RCp;

        // Step 7: Apply R and T to the entire Q point cloud
        List<Vector3> transformedQ = TransformPointCloud(Q, R, T);


        // Calculate Euclidean distances and Error
        float leastSquaresError = CalculateError(P, transformedQ);

        // Count inliers based on a threshold
        int theNumberOfInliers = CountInliers(P, transformedQ); 

        return (leastSquaresError, theNumberOfInliers, R, T, transformedQ);
    }


    // Calculate Euclidean distances between corresponding points in P and Q
    private List<float> CalculateDistances(List<Vector3> P, List<Vector3> transformedQ)
    {
        List<float> distances = new List<float>();

        // If both point clouds have the same number of points, calculate the distances between corresponding points
        if (P.Count == transformedQ.Count) {
            for (int i = 0; i < P.Count; i++)
            {
                // Euclidean distance: sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
                float distance = Vector3.Distance(P[i], transformedQ[i]);
                distances.Add(distance);
            }

        }

        // If the point clouds have different sizes, find the nearest neighbor for each point in transformedQ
        else
        {
            foreach (var transformedPoint in transformedQ)
            {
                float minDistance = float.MaxValue;

                // Find the closest point in P
                foreach (var point in P)
                {
                    float distance = Vector3.Distance(transformedPoint, point);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }

                distances.Add(minDistance);  // Add the minimum distance for this point
            }
        }


        return distances;
    }


    private int CountInliers(List<Vector3> P, List<Vector3> transformedQ)
    {
        int theNumberOfInliers = 0;

        // If the point clouds have the same number of points, directly compare corresponding points
        if (P.Count == transformedQ.Count)
        {
            for (int i = 0; i < P.Count; i++)
            {
                // Calculate the Euclidean distance
                float distance = Vector3.Distance(P[i], transformedQ[i]);

                // Count as inlier if the distance is below or equal to the threshold
                if (distance <= threshold)
                {
                    theNumberOfInliers++;
                }
            }
        }
        else
        {
            // If the point clouds have different sizes, find the nearest neighbor for each point in transformedQ
            foreach (var transformedPoint in transformedQ)
            {
                float minDistance = float.MaxValue;

                // Find the closest point in P
                foreach (var point in P)
                {
                    float distance = Vector3.Distance(transformedPoint, point);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }

                // Count as inlier if the nearest neighbor distance is below or equal to the threshold
                if (minDistance <= threshold)
                {
                    theNumberOfInliers++;
                }
            }
        }

        return theNumberOfInliers;
    }


    // Calculate error
    private float CalculateError(List<Vector3> P, List<Vector3> transformedQ)
    {
        List<float> distances = CalculateDistances(P, transformedQ);

        if (distances == null || distances.Count == 0)
        {
            Debug.LogError("Failed to calculate distances. Cannot compute error.");
            return float.MaxValue;
        }

        float sumOfSquares = 0f;

        foreach (float distance in distances)
        {
            sumOfSquares += distance * distance;
        }

        // Return mean squared error
        return sumOfSquares / distances.Count;
    }


    public void UpdateTranslationText(Vector3 translationVector)
    {
        if (TranslationText != null)
        {
            // Format the Vector3 values and update the text
            string formattedText = string.Format("Translation:\n[{0:F2}, {1:F2}, {2:F2}]",
                                                  translationVector.x,
                                                  translationVector.y,
                                                  translationVector.z);

            // Set the font size to 17
            TranslationText.fontSize = 17;

            TranslationText.fontStyle = FontStyle.Bold;
            TranslationText.text = formattedText;
        }
        else
        {
            Debug.LogError("TranslationText reference is missing!");
        }
    }

    public void UpdateRotationText(Matrix3x3 rotationMatrix)
    {
        if (RotationText != null)
        {
            // Format the 3x3 rotation matrix values into a string
            string formattedText = "Rotation Matrix:\n";

            // Loop through each row of the matrix
            for (int i = 0; i < 3; i++)
            {
                formattedText += "["; // Start the row with a bracket

                for (int j = 0; j < 3; j++)
                {
                    formattedText += rotationMatrix.values[i, j].ToString("F2") + ", "; // Add the values
                }

                formattedText = formattedText.TrimEnd(); // Remove the extra space at the end of the row

                // Remove the last comma
                if (formattedText.EndsWith(","))
                {
                    formattedText = formattedText.Remove(formattedText.Length - 1);
                }

                formattedText += "]\n"; // Close the row with a bracket
            }

            RotationText.fontSize = 17;
            RotationText.fontStyle = FontStyle.Bold;

            // Update the rotation text in the TranslationText object
            RotationText.text = formattedText;
        }
        else
        {
            Debug.LogError("RotationText reference is missing!");
        }
    }


    //Draw lines between original and transformed points
    public void DrawLinesBetweenPoints()
    {

        // Check if the 'PointCloudConnections' object already exists
        GameObject existingConnections = GameObject.Find("PointCloudConnections");
        if (existingConnections != null)
        {
            return;
        }


        // Find parent objects for the original and transformed point clouds
        GameObject originalParent = GameObject.Find("PointCloud2");
        GameObject transformedParent = GameObject.Find("TransformedPointCloud");

        if (originalParent == null)
        {
            Debug.LogError($"Point cloud 'PointCloud2' not found in the scene.");
            return;
        }

        if (transformedParent == null)
        {
            Debug.LogError($"Transformed point cloud 'TransformedPointCloud' not found in the scene.");
            return;
        }

        // Retrieve all child points from both parent objects
        Transform[] originalPoints = originalParent.GetComponentsInChildren<Transform>();
        Transform[] transformedPoints = transformedParent.GetComponentsInChildren<Transform>();

        // Validate the number of points
        if (originalPoints.Length != transformedPoints.Length)
        {
            Debug.LogError("The number of points in the original and transformed point clouds do not match.");
            return;
        }

        // Create a parent object for the lines
        GameObject lineParent = new GameObject("PointCloudConnections");

        // Iterate through the points and draw lines
        for (int i = 1; i < originalPoints.Length; i++) // Start at 1 to skip the parent objects themselves
        {
            Transform originalPoint = originalPoints[i];
            Transform transformedPoint = transformedPoints[i];

            // Create a line object
            GameObject lineObject = new GameObject($"Line_{i}");
            lineObject.transform.parent = lineParent.transform;

            LineRenderer lineRenderer = lineObject.AddComponent<LineRenderer>();
            lineRenderer.startWidth = 0.06f;
            lineRenderer.endWidth = 0.06f;
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.startColor = Color.magenta;
            lineRenderer.endColor = Color.magenta;

            // Set positions for the line
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, originalPoint.position);
            lineRenderer.SetPosition(1, transformedPoint.position);
        }

    }


    // Function to clean all game objects with the name "Sphere"
    public void CleanTransformedCloudAndLine()
    {
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
            sphere.name = "TransformedSphere" + i.ToString();
            i++;

            // Set the parent of the sphere to the created parent object
            sphere.transform.SetParent(parentObject.transform);
        }
    }

    private List<Vector3> TransformPointCloud(List<Vector3> pointCloud, Matrix3x3 R, Vector3 T)
    {
        List<Vector3> transformedPoints = new List<Vector3>();

        foreach (var point in pointCloud)
        {
            // Apply the transformation: Q' = R * Q + T
            Vector3 rotatedPoint = R.Multiply(point);
            Vector3 transformedPoint = rotatedPoint + T;
            transformedPoints.Add(transformedPoint);
        }

        return transformedPoints;
    }

    Matrix3x3 ConstructHMatrix(List<Vector3> centeredP, List<Vector3> centeredQ)
    {
        Matrix3x3 H = new Matrix3x3();

        for (int i = 0; i < centeredP.Count; i++)
        {
            Vector3 PPrime = centeredP[i];
            Vector3 QPrime = centeredQ[i];

            // Construct outer product QPrime * PPrime^T
            Matrix3x3 outerProduct = Matrix3x3.OuterProduct(QPrime, PPrime);  

            H = H.Add(outerProduct);
        }

        return H;
    }

    Matrix3x3 PerformSVD(Matrix3x3 H)
    {
        // Convert custom Matrix3x3 to Math.NET DenseMatrix
        var mathNetMatrix = DenseMatrix.OfArray(new double[,]
        {
        { H.values[0, 0], H.values[0, 1], H.values[0, 2] },
        { H.values[1, 0], H.values[1, 1], H.values[1, 2] },
        { H.values[2, 0], H.values[2, 1], H.values[2, 2] }
        });

        // Perform SVD using Math.NET Numerics library
        var svd = mathNetMatrix.Svd();

        // Extract U, S, and V^T matrices
        var U = svd.U;          // Left singular vectors
        var Vt = svd.VT;        // Transposed right singular vectors


        // Compute Rotation Matrix R = V * U^T
        var RMatrix = Vt.Transpose() * U.Transpose();

        double determinant = RMatrix.Determinant();

        if (determinant < 0)
        {
            // Multiply the last column of R Matrix by -1
            for (int i = 0; i < 3; i++)
            {
                RMatrix[i, 2] *= -1;
            }
        }

        // Convert RMatrix back to custom Matrix3x3
        Matrix3x3 R = new Matrix3x3();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R.values[i, j] = (float)RMatrix[i, j];
            }
        }

        return R;
    }

    List<Vector3> CenterPoints(List<Vector3> points, Vector3 centroid)
    {
        List<Vector3> centeredPoints = new List<Vector3>();
        foreach (var point in points)
        {
            centeredPoints.Add(point - centroid);
        }
        return centeredPoints;
    }

    Vector3 ComputeCentroid(List<Vector3> points)
    {
        Vector3 centroid = Vector3.zero;
        foreach (var point in points)
        {
            centroid += point;
        }
        return centroid / points.Count;
    }

    List<Vector3> ReadPointsFromFile(string path)
    {
        List<Vector3> points = new List<Vector3>();

        using (StreamReader reader = new StreamReader(path))
        {
            string line = reader.ReadLine();  // First line is the number of points
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

    List<Vector3> SelectRandomPoints(List<Vector3> points, int count)
    {
        List<Vector3> selectedPoints = new List<Vector3>();
        HashSet<int> indices = new HashSet<int>();

        while (selectedPoints.Count < count)
        {
            int randomIndex = Random.Range(0, points.Count);
            if (!indices.Contains(randomIndex))
            {
                indices.Add(randomIndex);
                selectedPoints.Add(points[randomIndex]);
            }
        }

        return selectedPoints;
    }
}


public class Matrix3x3
{
    public float[,] values;

    public Matrix3x3()
    {
        values = new float[3, 3];
    }

    // Function to calculate the determinant of the 3x3 matrix
    public float Determinant()
    {
        float a = values[0, 0], b = values[0, 1], c = values[0, 2];
        float d = values[1, 0], e = values[1, 1], f = values[1, 2];
        float g = values[2, 0], h = values[2, 1], i = values[2, 2];

        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    }

    public static Matrix3x3 OuterProduct(Vector3 a, Vector3 b)
    {
        Matrix3x3 result = new Matrix3x3();

        // Multiplication is done assuming a is 3x1 matrix (Vector3),  b is 1x3 matrix (Transposed Vector3)
        result.values[0, 0] = a.x * b.x;
        result.values[0, 1] = a.x * b.y;
        result.values[0, 2] = a.x * b.z;

        result.values[1, 0] = a.y * b.x;
        result.values[1, 1] = a.y * b.y;
        result.values[1, 2] = a.y * b.z;

        result.values[2, 0] = a.z * b.x;
        result.values[2, 1] = a.z * b.y;
        result.values[2, 2] = a.z * b.z;

        return result;
    }


    public Matrix3x3 Add(Matrix3x3 other)
    {
        Matrix3x3 result = new Matrix3x3();

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                result.values[i, j] = this.values[i, j] + other.values[i, j];
            }
        }

        return result;
    }


    public Vector3 Multiply(Vector3 vector)
    {
        Vector3 result = new Vector3();

        result.x = values[0, 0] * vector.x + values[0, 1] * vector.y + values[0, 2] * vector.z;
        result.y = values[1, 0] * vector.x + values[1, 1] * vector.y + values[1, 2] * vector.z;
        result.z = values[2, 0] * vector.x + values[2, 1] * vector.y + values[2, 2] * vector.z;

        return result;
    }

    public override string ToString()
    {
        string matrixString = "";
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                matrixString += values[i, j].ToString("F2") + " ";
            }
            matrixString += "\n";
        }
        return matrixString;
    }
}