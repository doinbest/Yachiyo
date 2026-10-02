using System.Collections;

namespace Xunit;

[AttributeUsage(AttributeTargets.Method)]
public sealed class FactAttribute : Attribute;

public static class Assert
{
    public static void True(bool condition, string? message = null)
    {
        if (!condition) throw new AssertionException(message ?? "Expected true, got false.");
    }

    public static void False(bool condition, string? message = null) =>
        True(!condition, message ?? "Expected false, got true.");

    public static void Equal<T>(T expected, T actual)
    {
        if (expected is IEnumerable expectedItems && actual is IEnumerable actualItems &&
            expected is not string && actual is not string)
        {
            var left = expectedItems.Cast<object?>().ToArray();
            var right = actualItems.Cast<object?>().ToArray();
            if (!left.SequenceEqual(right))
            {
                throw new AssertionException($"Expected [{string.Join(", ", left)}], got [{string.Join(", ", right)}].");
            }

            return;
        }

        if (!EqualityComparer<T>.Default.Equals(expected, actual))
        {
            throw new AssertionException($"Expected {expected}, got {actual}.");
        }
    }

    public static void NotEmpty(IEnumerable values)
    {
        var enumerator = values.GetEnumerator();
        if (!enumerator.MoveNext()) throw new AssertionException("Expected a non-empty collection.");
    }

    public static void Empty(IEnumerable values)
    {
        var enumerator = values.GetEnumerator();
        if (enumerator.MoveNext()) throw new AssertionException("Expected an empty collection.");
    }

    public static T Single<T>(IEnumerable<T> values)
    {
        var items = values.ToArray();
        if (items.Length != 1) throw new AssertionException($"Expected one item, got {items.Length}.");
        return items[0];
    }

    public static void Null(object? value)
    {
        if (value is not null) throw new AssertionException($"Expected null, got {value}.");
    }

    public static T NotNull<T>(T? value) where T : class =>
        value ?? throw new AssertionException("Expected a non-null value.");

    public static TException Throws<TException>(Action action) where TException : Exception
    {
        try
        {
            action();
        }
        catch (TException exception)
        {
            return exception;
        }

        throw new AssertionException($"Expected {typeof(TException).Name} to be thrown.");
    }

    public static async Task<TException> ThrowsAsync<TException>(Func<Task> action) where TException : Exception
    {
        try
        {
            await action();
        }
        catch (TException exception)
        {
            return exception;
        }

        throw new AssertionException($"Expected {typeof(TException).Name} to be thrown.");
    }

    public static void Contains<T>(IEnumerable<T> values, Predicate<T> predicate)
    {
        if (!values.Any(value => predicate(value)))
        {
            throw new AssertionException("Expected a matching item in the collection.");
        }
    }

    public static void Contains(string expectedSubstring, string actual)
    {
        if (!actual.Contains(expectedSubstring, StringComparison.Ordinal))
        {
            throw new AssertionException($"Expected '{actual}' to contain '{expectedSubstring}'.");
        }
    }
}

public sealed class AssertionException(string message) : Exception(message);
