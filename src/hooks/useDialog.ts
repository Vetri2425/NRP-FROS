import { useState, useCallback } from 'react';

export interface DialogConfig {
  type: 'alert' | 'confirm';
  title: string;
  message: string;
  confirmText?: string;
  cancelText?: string;
}

export interface DialogState extends DialogConfig {
  isOpen: boolean;
  onConfirm?: () => void;
  onCancel?: () => void;
}

export const useDialog = () => {
  const [dialogState, setDialogState] = useState<DialogState>({
    isOpen: false,
    type: 'alert',
    title: '',
    message: '',
  });

  const showAlert = useCallback((title: string, message: string) => {
    return new Promise<void>((resolve) => {
      setDialogState({
        isOpen: true,
        type: 'alert',
        title,
        message,
        onConfirm: () => {
          setDialogState(prev => ({ ...prev, isOpen: false }));
          resolve();
        },
      });
    });
  }, []);

  const showConfirm = useCallback((title: string, message: string, confirmText = 'OK', cancelText = 'Cancel') => {
    return new Promise<boolean>((resolve) => {
      setDialogState({
        isOpen: true,
        type: 'confirm',
        title,
        message,
        confirmText,
        cancelText,
        onConfirm: () => {
          setDialogState(prev => ({ ...prev, isOpen: false }));
          resolve(true);
        },
        onCancel: () => {
          setDialogState(prev => ({ ...prev, isOpen: false }));
          resolve(false);
        },
      });
    });
  }, []);

  const closeDialog = useCallback(() => {
    setDialogState(prev => ({ ...prev, isOpen: false }));
  }, []);

  return {
    dialogState,
    showAlert,
    showConfirm,
    closeDialog,
  };
};