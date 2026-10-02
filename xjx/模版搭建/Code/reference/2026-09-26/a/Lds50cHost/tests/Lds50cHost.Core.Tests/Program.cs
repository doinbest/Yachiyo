using System.Reflection;
using Xunit;

var filter = args.FirstOrDefault();
var tests = Assembly.GetExecutingAssembly()
    .GetTypes()
    .OrderBy(type => type.FullName)
    .SelectMany(type => type.GetMethods()
        .Where(method => method.GetCustomAttribute<FactAttribute>() is not null)
        .Select(method => (Type: type, Method: method)))
    .Where(test => filter is null ||
        $"{test.Type.FullName}.{test.Method.Name}".Contains(filter, StringComparison.OrdinalIgnoreCase))
    .ToArray();

var failures = new List<string>();
foreach (var test in tests)
{
    var name = $"{test.Type.Name}.{test.Method.Name}";
    try
    {
        var instance = test.Method.IsStatic ? null : Activator.CreateInstance(test.Type);
        var result = test.Method.Invoke(instance, null);
        if (result is Task task) await task;
        Console.WriteLine($"PASS {name}");
    }
    catch (Exception exception)
    {
        var cause = exception is TargetInvocationException { InnerException: not null }
            ? exception.InnerException
            : exception;
        failures.Add($"FAIL {name}: {cause!.GetType().Name}: {cause.Message}");
        Console.WriteLine(failures[^1]);
    }
}

Console.WriteLine($"RESULT {tests.Length - failures.Count}/{tests.Length} passed");
return failures.Count == 0 ? 0 : 1;
