using Lds50cHost.Core.RadarProtocol;

namespace Lds50cHost.Services;

public sealed record RadarStatusSnapshot(
    bool UsesMillimetres,
    bool EnergyEnabled,
    bool TrailingPointRemovalEnabled,
    bool FilterEnabled,
    DateTimeOffset ReceivedAt)
{
    public static RadarStatusSnapshot From(StatusFrame frame, DateTimeOffset receivedAt)
    {
        ArgumentNullException.ThrowIfNull(frame);
        return new RadarStatusSnapshot(
            frame.UsesMillimetres,
            frame.EnergyEnabled,
            frame.TrailingPointRemovalEnabled,
            frame.FilterEnabled,
            receivedAt);
    }
}
