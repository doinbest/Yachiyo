using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class ContinuousScanLeaseOwnerTests
{
    [Fact]
    public void DisposeBeforeAdopt_RejectsTheLateLease()
    {
        var owner = new ContinuousScanLeaseOwner();

        var ownedAtDisposal = owner.DisposeAndTake();
        var adoptedAfterDisposal = owner.TryAdopt(new ContinuousScanLease(17));

        Assert.Null(ownedAtDisposal);
        Assert.False(adoptedAfterDisposal);
    }

    [Fact]
    public void AdoptBeforeDispose_ReturnsTheOwnedLeaseExactlyOnce()
    {
        var owner = new ContinuousScanLeaseOwner();
        var lease = new ContinuousScanLease(23);

        Assert.True(owner.TryAdopt(lease));
        Assert.Equal(lease, owner.DisposeAndTake());
        Assert.Null(owner.DisposeAndTake());
    }
}
