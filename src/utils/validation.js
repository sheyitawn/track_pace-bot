export function validateSplits(splits) {
    if (!splits.length) return "Please add at least one split.";
    const sorted = [...splits].sort((a, b) => a.start - b.start);
    for (let i = 0; i < sorted.length; i++) {
        const s = sorted[i];
        if (s.end <= s.start) return `Split ${i + 1}: end must be > start.`;
        if (s.time <= 0) return `Split ${i + 1}: time must be > 0.`;
        if (i > 0 && s.start !== sorted[i - 1].end) {
            return `Split ${i + 1}: start (${s.start}) must equal previous end (${sorted[i - 1].end}).`;
        }
    }
    const total = sorted[sorted.length - 1].end - sorted[0].start;
    if (total <= 0) return "Total distance must be positive.";
    return null;
}
