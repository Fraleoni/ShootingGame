
uniform sampler2D myTexture;

void main (void)
{
    vec4 glColor = vec4(1, 1, 0, 1);
    vec4 col = texture2D(myTexture, vec2(.5));
    col *= glColor;
    gl_FragColor = col * 4.0;
}
