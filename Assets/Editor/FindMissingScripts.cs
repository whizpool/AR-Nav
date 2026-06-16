using UnityEditor;
using UnityEngine;

public class FindMissingScripts : EditorWindow
{
    [MenuItem("Tools/Find Missing Scripts In Scene")]
    static void FindMissing()
    {
        int count = 0;
        foreach (GameObject go in Resources.FindObjectsOfTypeAll<GameObject>())
        {
            foreach (Component c in go.GetComponents<Component>())
            {
                if (c == null)
                {
                    Debug.LogError($"Missing script on: '{go.name}' " +
                        $"in scene: '{go.scene.name}'", go);
                    count++;
                }
            }
        }
        Debug.Log($"Found {count} missing scripts total.");
    }

    [MenuItem("Tools/Find Missing Scripts In Prefabs")]
    static void FindMissingInPrefabs()
    {
        string[] guids = AssetDatabase.FindAssets("t:Prefab");
        int count = 0;
        foreach (string guid in guids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            if (prefab == null) continue;
            foreach (Component c in prefab.GetComponentsInChildren<Component>(true))
            {
                if (c == null)
                {
                    Debug.LogError($"Missing script in prefab: {path}", prefab);
                    count++;
                }
            }
        }
        Debug.Log($"Found {count} missing scripts in prefabs.");
    }
}