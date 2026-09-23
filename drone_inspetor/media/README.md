# Propriedade dos recursos de mídia

`VideoRecorder` é o único proprietário de `cv2.VideoWriter`. Abertura, escrita e
fechamento usam o mesmo lock; fechar aguarda a escrita ativa, uma segunda abertura
não abandona o arquivo anterior e falhas liberam o recurso. A câmera abre no
primeiro frame; o CV abre na resolução fixa de 1280×720. Frames com outra resolução
são redimensionados. O status da câmera só passa a gravando após abertura efetiva.
A taxa FPS do arquivo deve corresponder à taxa real dos frames fornecidos: não há
reamostragem temporal nem inserção de frames duplicados.

`save_photo` centraliza compressão e verifica o retorno booleano de `cv2.imwrite`.
`filename_component` impede que nomes de objetos sejam usados como subdiretórios.
Falhas são reportadas ao nó, em vez de anunciarem uma captura inexistente.

## Integração com percepção

O CV separa inferência (`inference.py`), catálogo/troca de modelos
(`model_registry.py`) e espera por observações (`detection_buffer.py`). Pesos e
nomes de classes são consistentes durante cada frame. A troca do par de modelos
só é efetivada depois que ambos carregam; uma falha conserva o par anterior.
`inference_device=auto` seleciona GPU disponível ou CPU de fato.

`detection_max_frame_age_seconds` (padrão 1.0s) limita a idade do frame. Uma consulta
exige observação adquirida após seu início, e o timeout utiliza relógio monotônico,
independente de `/clock`. Um novo ponto/missão ou troca de modelos invalida consultas
pendentes. O encerramento acorda os serviços antes de aguardar o executor.

Processamento de imagem, estado de missão e controles curtos usam o mesmo grupo
exclusivo de callbacks. A consulta bloqueante de detecção tem grupo independente.
Inferência e gravação permanecem síncronas: modelos lentos podem reduzir a taxa
real de vídeo e atrasar controles até o fim do frame atual. A inferência nativa em
GPU não oferece cancelamento forçado; shutdown aguarda o processamento corrente.

Os testes com writers e preditores falsos exercitam concorrência, falhas,
substituição de modelos e freshness sem GPU. Os testes de callbacks usam requests e
mensagens ROS reais sem abrir nós. Hardware, codecs disponíveis no deployment e
latência de inferência ainda exigem validação integrada.
