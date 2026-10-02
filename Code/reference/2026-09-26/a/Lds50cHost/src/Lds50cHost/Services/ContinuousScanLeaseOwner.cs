namespace Lds50cHost.Services;

public sealed class ContinuousScanLeaseOwner
{
    private readonly object _gate = new();
    private ContinuousScanLease? _lease;
    private bool _disposed;

    public bool TryAdopt(ContinuousScanLease lease)
    {
        lock (_gate)
        {
            if (_disposed) return false;
            if (_lease is not null)
                throw new InvalidOperationException("This component already owns a continuous scan session.");

            _lease = lease;
            return true;
        }
    }

    public void Clear()
    {
        lock (_gate) _lease = null;
    }

    public ContinuousScanLease? DisposeAndTake()
    {
        lock (_gate)
        {
            _disposed = true;
            var lease = _lease;
            _lease = null;
            return lease;
        }
    }
}
