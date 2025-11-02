import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { ProgressBar } from '../components/ProgressBar';

describe('ProgressBar', () => {
  it('should render with correct percentage', () => {
    render(<ProgressBar progress={50} />);
    const progressbar = screen.getByRole('progressbar');
    expect(progressbar).toHaveAttribute('aria-valuenow', '50');
  });

  it('should display percentage text when showPercentage is true', () => {
    render(<ProgressBar progress={75} showPercentage={true} />);
    expect(screen.getByText('75%')).toBeInTheDocument();
  });

  it('should not display percentage text when showPercentage is false', () => {
    render(<ProgressBar progress={75} showPercentage={false} />);
    expect(screen.queryByText('75%')).not.toBeInTheDocument();
  });

  it('should have animated class when animated prop is true', () => {
    const { container } = render(<ProgressBar progress={50} animated={true} />);
    expect(container.querySelector('.progress-bar.animated')).toBeInTheDocument();
  });

  it('should not have animated class when animated prop is false', () => {
    const { container } = render(<ProgressBar progress={50} animated={false} />);
    expect(container.querySelector('.progress-bar.animated')).not.toBeInTheDocument();
  });

  it('should have correct status class for loading', () => {
    const { container } = render(<ProgressBar progress={50} status="loading" />);
    expect(container.querySelector('.progress-loading')).toBeInTheDocument();
  });

  it('should have correct status class for success', () => {
    const { container } = render(<ProgressBar progress={100} status="success" />);
    expect(container.querySelector('.progress-success')).toBeInTheDocument();
  });

  it('should have correct status class for error', () => {
    const { container } = render(<ProgressBar progress={25} status="error" />);
    expect(container.querySelector('.progress-error')).toBeInTheDocument();
  });

  it('should clamp progress to 0-100 range', () => {
    const { rerender } = render(<ProgressBar progress={-10} />);
    let progressbar = screen.getByRole('progressbar');
    expect(progressbar).toHaveAttribute('aria-valuenow', '0');

    rerender(<ProgressBar progress={150} />);
    progressbar = screen.getByRole('progressbar');
    expect(progressbar).toHaveAttribute('aria-valuenow', '100');
  });

  it('should be accessible with ARIA attributes', () => {
    render(
      <ProgressBar 
        progress={50} 
        label="Upload Progress"
      />
    );
    const progressbar = screen.getByRole('progressbar');
    expect(progressbar).toHaveAttribute('aria-label', 'Upload Progress');
    expect(progressbar).toHaveAttribute('aria-valuemin', '0');
    expect(progressbar).toHaveAttribute('aria-valuemax', '100');
    expect(progressbar).toHaveAttribute('aria-valuenow', '50');
  });

  it('should display label when provided', () => {
    render(<ProgressBar progress={50} label="Loading..." />);
    expect(screen.getByText('Loading...')).toBeInTheDocument();
  });

  it('should apply correct size class', () => {
    const { container, rerender } = render(<ProgressBar progress={50} size="small" />);
    expect(container.querySelector('.progress-small')).toBeInTheDocument();

    rerender(<ProgressBar progress={50} size="medium" />);
    expect(container.querySelector('.progress-medium')).toBeInTheDocument();

    rerender(<ProgressBar progress={50} size="large" />);
    expect(container.querySelector('.progress-large')).toBeInTheDocument();
  });

  it('should default to medium size when size prop is not provided', () => {
    const { container } = render(<ProgressBar progress={50} />);
    expect(container.querySelector('.progress-medium')).toBeInTheDocument();
  });

  it('should have correct width style on progress fill', () => {
    const { container } = render(<ProgressBar progress={65} />);
    const progressFill = container.querySelector('.progress-fill') as HTMLElement;
    expect(progressFill).toHaveStyle({ width: '65%' });
  });
});
