using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public class InstanceMap
{
    static Dictionary<int, object> map = new Dictionary<int, object>();
    static int nextID = 1;
    public static int Add(object obj)
    {
        map.Add(nextID, obj);
        nextID++;
        return nextID - 1;
    }
    public static object GetObject(int id)
    {
        object obj;
        if(map.TryGetValue(id, out obj)) {
            return obj;
        }
        return null;
    }
    public static void Remove(int id)
    {
        map.Remove(id);
    }
}