import { useState, useEffect, useRef } from 'react';

const LERP_FACTOR = 0.1; // Linear interpolation factor for smoothing

/**
 * A hook that smoothly interpolates a heading (in degrees), handling the wrap-around from 360 to 0.
 * @param targetHeading The target heading in degrees.
 * @param factor The interpolation factor.
 * @returns The current smoothed heading.
 */
export function useSmoothedHeading(targetHeading: number, factor: number = LERP_FACTOR): number {
    const [currentHeading, setCurrentHeading] = useState(targetHeading);
    const animationFrameRef = useRef<number | null>(null);

    useEffect(() => {
        const animate = () => {
            setCurrentHeading(prev => {
                let difference = targetHeading - prev;

                // Handle the wrap-around from 360 to 0 degrees
                if (difference > 180) {
                    difference -= 360;
                } else if (difference < -180) {
                    difference += 360;
                }

                if (Math.abs(difference) < 0.01) {
                    if (animationFrameRef.current) {
                        cancelAnimationFrame(animationFrameRef.current);
                    }
                    return targetHeading;
                }
                
                const nextHeading = prev + difference * factor;
                return nextHeading;
            });
            animationFrameRef.current = requestAnimationFrame(animate);
        };

        animationFrameRef.current = requestAnimationFrame(animate);

        return () => {
            if (animationFrameRef.current) {
                cancelAnimationFrame(animationFrameRef.current);
            }
        };
    }, [targetHeading, factor]);

    // Normalize heading to be within [0, 360)
    const normalizedHeading = ((currentHeading % 360) + 360) % 360;

    return normalizedHeading;
}
