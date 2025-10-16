/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_JETSON_BACKEND_URL: string;
  // You can add other environment variables here as needed
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
