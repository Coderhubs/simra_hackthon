// This is a server-side API route that acts as a proxy to the backend RAG service
// This helps avoid CORS issues when the frontend tries to call the backend directly

export default async function handler(req, res) {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Credentials', true);
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,OPTIONS,PATCH,DELETE,POST,PUT');
  res.setHeader(
    'Access-Control-Allow-Headers',
    'X-CSRF-Token, X-Requested-With, Accept, Accept-Version, Content-Length, Content-MD5, Content-Type, Date, X-Api-Version'
  );

  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  if (req.method !== 'POST') {
    res.status(405).json({ error: 'Method not allowed' });
    return;
  }

  try {
    const { query, selected_text, filters, top_k } = req.body;

    // Forward the request to the backend RAG service
    const backendResponse = await fetch('http://localhost:8002/api/v1/search-live-content', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query,
        selected_text,
        filters,
        top_k: top_k || 5,
      }),
    });

    if (!backendResponse.ok) {
      throw new Error(`Backend error: ${backendResponse.status} ${backendResponse.statusText}`);
    }

    const data = await backendResponse.json();

    res.status(200).json(data);
  } catch (error) {
    console.error('Proxy error:', error);
    res.status(500).json({
      error: 'Failed to fetch from backend service',
      details: error.message
    });
  }
}