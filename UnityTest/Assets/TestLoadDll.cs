using UnityEngine;
using System.Collections;
using System;
using System.Reflection;
using System.Security.Policy;

class Loader : MarshalByRefObject
{
    private Assembly _assembly;

    public override object InitializeLifetimeService()
    {
        return null;
    }

    public void LoadAssembly(string path)
    {
        _assembly = Assembly.Load(AssemblyName.GetAssemblyName(path));
    }

    public object ExecuteStaticMethod(string typeName, string methodName, params object[] parameters)
    {
        Type type = _assembly.GetType(typeName);
        // TODO: this won't work if there are overloads available
        MethodInfo method = type.GetMethod(
            methodName,
            BindingFlags.Static | BindingFlags.Public);
        return method.Invoke(null, parameters);
    }
}

public class TestLoadDll : MonoBehaviour
{
    AppDomain domain;
    // Use this for initialization
    void Start()
    {
        string dllPath = Application.dataPath + "\\Plugins\\Windows\\x86_64\\ode.dll";

        var friendlyName = "PlayerAppDomain";
        var assembly = Assembly.GetExecutingAssembly();
        var codeBase = assembly.Location;
        var codeBaseDirectory = System.IO.Path.GetDirectoryName(codeBase);
        var info = new AppDomainSetup() {
            ApplicationName = "123",
            ApplicationBase = codeBaseDirectory,
            DynamicBase = codeBaseDirectory,
        };
        this.domain = AppDomain.CreateDomain(friendlyName, new System.Security.Policy.Evidence(), info);

        //domain = AppDomain.CreateDomain("Test");

        // Loader lives in another AppDomain
        Loader loader = (Loader)domain.CreateInstanceAndUnwrap(
            typeof(Loader).Assembly.FullName,
            typeof(Loader).FullName);

        loader.LoadAssembly(dllPath);
        //loader.ExecuteStaticMethod(
        //    "ClassLibrary1.Class1",
        //    "DoStuff",
        //    DateTime.Now.ToShortDateString());
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void OnDestroy()
    {
        AppDomain.Unload(domain);
    }
}
